#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Port_Ethernet.h"
#include "AP_Networking_ChibiOS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

extern const AP_HAL::HAL& hal;

#define LWIP_SEND_TIMEOUT_MS 50

bool AP_Networking_Port_Ethernet::init()
{
    if (!AP_Networking_ChibiOS::allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate MAC buffers");
        return false;
    }
    if (!mac_init_and_start()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: MAC init/start failed");
        return false;
    }
    // allocate TX queue
    if (tx_queue == nullptr) {
        tx_queue = NEW_NOTHROW ByteBuffer(TX_QUEUE_SIZE);
        if (tx_queue == nullptr) {
            return false;
        }
    }
    // start dedicated RX thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Port_Ethernet::rx_thread, void),
                                      "eth_rx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_rx thread start failed");
        return false;
    }
    // start dedicated TX thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Port_Ethernet::tx_thread, void),
                                      "eth_tx",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: eth_tx thread start failed");
        return false;
    }
    return true;
}

bool AP_Networking_Port_Ethernet::mac_init_and_start()
{
    if (!macInit()) {
        return false;
    }

    // MAC address for hardware
    const MACConfig mac_config = {macaddr};
    macStart(&ETHD1, &mac_config);

    set_promiscuous_mode();
    return true;
}

void AP_Networking_Port_Ethernet::set_promiscuous_mode()
{
#if defined(LWIP_IGMP) && LWIP_IGMP
    if (ETH != nullptr) {
        // permit multicast
        ETH->MACPFR |= ETH_MACPFR_PM;
    }
#endif
    if (ETH != nullptr) {
#ifdef ETH_MACPFR_PR
        ETH->MACPFR |= ETH_MACPFR_PR; // promiscuous receive
#endif
    }
}

void AP_Networking_Port_Ethernet::update()
{
    // nothing required; threads handle RX/TX
}

bool AP_Networking_Port_Ethernet::process_one_rx_descriptor(sysinterval_t timeout)
{
    MACReceiveDescriptor rd;
    if (macWaitReceiveDescriptor(&ETHD1, &rd, timeout) != MSG_OK) {
        return false;
    }
    const size_t len = (size_t)rd.size;
    if (len == 0 || len > MAX_FRAME) {
        macReleaseReceiveDescriptorX(&rd);
        rx_errors++;
        return true;
    }
    macReadReceiveDescriptor(&rd, rx_framebuf, len);
    macReleaseReceiveDescriptorX(&rd);
    hub->route_frame(this, rx_framebuf, len);
    rx_count++;
    return true;
}

void AP_Networking_Port_Ethernet::rx_thread()
{
    while (true) {
        if (!process_one_rx_descriptor(TIME_INFINITE)) {
            continue;
        }
        while (process_one_rx_descriptor(TIME_IMMEDIATE)) {
        }
    }
}

void AP_Networking_Port_Ethernet::tx_thread()
{
    static uint8_t buf[MAX_FRAME];
    while (true) {
        if (tx_queue == nullptr) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }
        // Block until signaled
        tx_not_empty.wait_blocking();
        // Drain all available frames
        while (true) {
            tx_mutex.take_blocking();
            if (tx_queue->available() < 2) {
                tx_mutex.give();
                break;
            }
            uint8_t hdr[2];
            if (tx_queue->peekbytes(hdr, 2) != 2) {
                tx_mutex.give();
                break;
            }
            const uint16_t l = (uint16_t)(hdr[0] | (uint16_t(hdr[1]) << 8));
            if (l == 0 || l > MAX_FRAME) {
                (void)tx_queue->advance(tx_queue->available());
                tx_mutex.give();
                tx_errors++;
                break;
            }
            if (tx_queue->available() < (uint32_t)(2 + l)) {
                tx_mutex.give();
                break;
            }
            (void)tx_queue->advance(2);
            if (tx_queue->read(buf, l) != l) {
                tx_mutex.give();
                break;
            }
            tx_mutex.give();
            // Transmit via MAC (outside mutex)
            MACTransmitDescriptor td;
            if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
                tx_errors++;
                continue;
            }
            macWriteTransmitDescriptor(&td, buf, (size_t)l);
            macReleaseTransmitDescriptorX(&td);
            tx_count++;
        }
    }
}

void AP_Networking_Port_Ethernet::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > AP_Networking_Hub::MAX_ETH_FRAME) {
        tx_errors++;
        return;
    }
    if (tx_queue == nullptr) {
        return;
    }
    // Serialize writers; avoid blocking the caller
    if (!tx_mutex.take(1)) {
        // drop if busy
        tx_errors++;
        return;
    }
    const uint32_t need = (uint32_t)(2 + len);
    if (tx_queue->space() < need) {
        // drop if full
        tx_errors++;
        tx_mutex.give();
        return;
    }
    uint16_t l = (uint16_t)len;
    const uint8_t hdr[2] = { (uint8_t)(l & 0xFF), (uint8_t)((l >> 8) & 0xFF) };
    (void)tx_queue->write(hdr, 2);
    (void)tx_queue->write(frame, (uint32_t)len);
    tx_mutex.give();
    // Signal TX thread
    tx_not_empty.signal();
}

bool AP_Networking_Port_Ethernet::can_receive() const
{
    return tx_queue != nullptr && tx_queue->space() >= (2 + MAX_FRAME);
}

bool AP_Networking_Port_Ethernet::is_link_up() const
{
    return poll_link_status();
}

bool AP_Networking_Port_Ethernet::poll_link_status() const
{
    return macPollLinkStatus(&ETHD1);
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
