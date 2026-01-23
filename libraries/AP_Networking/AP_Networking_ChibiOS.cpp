
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_CHIBIOS

#include "AP_Networking_ChibiOS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
#include <lwip/etharp.h>
#include <hal.h>
#include "../../modules/ChibiOS/os/various/evtimer.h"
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

extern const AP_HAL::HAL& hal;

#ifdef NEEDS_KSZ9896C_ERRATA
#include <hal_mii.h>
#endif

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
#include "AP_Networking_Port_lwIP.h"
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
#include "AP_Networking_Port_Ethernet.h"
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
#include "AP_Networking_Port_COBS.h"
#endif

#define LWIP_SEND_TIMEOUT_MS 50
#define LWIP_NETIF_MTU       1500
#define LWIP_LINK_POLL_INTERVAL TIME_S2I(5)

#define PERIODIC_TIMER_ID       1
#define FRAME_RECEIVED_ID       2

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
#define LWIP_PORT_FRAME_ID      4
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
#define FAST_COBS_POLL_INTERVAL TIME_MS2I(1)
#define FAST_COBS_TIMER_ID       8
#endif

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
// Original upstream code path: ChibiOS backend owns the MAC directly
#ifndef STM32_ETH_BUFFERS_EXTERN
#error "Must use external ethernet buffers"
#endif
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#ifdef STM32_ETH_BUFFERS_EXTERN
/*
  these are referenced as globals inside lwip/MAC driver
*/
stm32_eth_rx_descriptor_t *__eth_rd;
stm32_eth_tx_descriptor_t *__eth_td;
uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

/*
  allocate buffers for LWIP/MAC DMA
*/
bool AP_Networking_ChibiOS::allocate_buffers()
{
#define AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE ((((STM32_MAC_BUFFERS_SIZE - 1) | 3) + 1) / 4)
    // check total size of buffers
    const uint32_t total_size = sizeof(stm32_eth_rx_descriptor_t)*STM32_MAC_RECEIVE_BUFFERS +
        sizeof(stm32_eth_tx_descriptor_t)*STM32_MAC_TRANSMIT_BUFFERS +
        sizeof(uint32_t)*STM32_MAC_RECEIVE_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE +
        sizeof(uint32_t)*STM32_MAC_TRANSMIT_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE;

    // ensure that we allocate 32-bit aligned memory, and mark it non-cacheable
    uint32_t size = 1;
    uint8_t rasr = 0;
    // find size closest to power of 2
    while (size < total_size) {
        size = size << 1;
        rasr++;
    }
    void *mem = malloc_eth_safe(size);
    if (mem == nullptr) {
        return false;
    }

#ifndef HAL_BOOTLOADER_BUILD
    // ensure our memory is aligned
    // ref. Cortex-M7 peripherals PM0253, section 4.6.4 MPU region base address register
    if (((uint32_t)mem) % size) {
        AP_HAL::panic("Bad alignment of ETH memory");
    }
#endif

    // for total_size close to 9240, size should be 16384 and (rasr-1) should be 13 (MPU_RASR_SIZE_16K)
    const uint32_t rasr_size = MPU_RASR_SIZE(rasr-1);

    // set up MPU region for buffers
    mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_ETH,
                       (uint32_t)mem,
                       MPU_RASR_ATTR_AP_RW_RW |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_ATTR_S |
                       rasr_size |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA);
    SCB_CleanInvalidateDCache();

    // assign buffers
    __eth_rd = (stm32_eth_rx_descriptor_t *)mem;
    __eth_td = (stm32_eth_tx_descriptor_t *)&__eth_rd[STM32_MAC_RECEIVE_BUFFERS];
    __eth_rb[0] = (uint32_t*)&__eth_td[STM32_MAC_TRANSMIT_BUFFERS];
    for (uint16_t i = 1; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
        __eth_rb[i] = &(__eth_rb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    __eth_tb[0] = &(__eth_rb[STM32_MAC_RECEIVE_BUFFERS-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    for (uint16_t i = 1; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {
        __eth_tb[i] = &(__eth_tb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    return true;
}
#else // !STM32_ETH_BUFFERS_EXTERN
bool AP_Networking_ChibiOS::allocate_buffers()
{
    return false;
}
#endif // STM32_ETH_BUFFERS_EXTERN

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
/*
  Process a received frame buffer and deliver to lwIP.
  Allocates a pbuf, copies data, and dispatches IP/ARP frames to netif.
*/
static void process_frame_to_lwip(const uint8_t *buf, size_t len, struct netif *netif)
{
    if (len == 0) {
        return;
    }
#if AP_NETWORKING_CAPTURE_ENABLED
    AP_Networking_ChibiOS::capture_frame(buf, len);
#endif
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p == nullptr) {
        return;
    }
#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);
#endif
    size_t ofs = 0;
    for (struct pbuf *q = p; q != nullptr; q = q->next) {
        size_t n = q->len;
        if (n > len - ofs) {
            n = len - ofs;
        }
        memcpy(q->payload, &buf[ofs], n);
        ofs += n;
        if (ofs >= len) {
            break;
        }
    }
#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);
#endif
    struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
    switch (htons(ethhdr->type)) {
    case ETHTYPE_IP:
    case ETHTYPE_ARP:
        if (netif->input(p, netif) == ERR_OK) {
            break;
        }
        /* Falls through */
    default:
        pbuf_free(p);
    }
}

// Static switch interface callbacks (optional)
static AP_Networking_ChibiOS::rx_get_frame_f s_switch_rx_cb = nullptr;
static AP_Networking_ChibiOS::tx_send_frame_f s_switch_tx_cb = nullptr;

void AP_Networking_ChibiOS::set_switch_interface(rx_get_frame_f rx_cb, tx_send_frame_f tx_cb)
{
    s_switch_rx_cb = rx_cb;
    s_switch_tx_cb = tx_cb;
}

bool AP_Networking_ChibiOS::switch_interface_active()
{
    return (s_switch_rx_cb != nullptr) && (s_switch_tx_cb != nullptr);
}
#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP

/*
  initialise ChibiOS network backend using LWIP
*/
bool AP_Networking_ChibiOS::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->pinMode(HAL_GPIO_ETH_ENABLE, HAL_GPIO_OUTPUT);
#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, frontend.param.enabled ? 1 : 0);
#else
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, 0); // reset
    hal.scheduler->delay(25);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, frontend.param.enabled ? 1 : 0);
    hal.scheduler->delay(10);
#endif
#endif

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    // Original path: ChibiOS backend initializes MAC directly
    if (!allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate buffers");
        return false;
    }

    if (!macInit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: macInit failed");
        return false;
    }

#if LWIP_IGMP
    if (ETH != nullptr) {
        // enable "permit multicast" so we can receive multicast packets
        ETH->MACPFR |= ETH_MACPFR_PM;
    }
#endif
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        return false;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_ChibiOS::thread, void),
                                      "network",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        return false;
    }
    
    return true;
}

void AP_Networking_ChibiOS::link_up_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_ChibiOS *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_start(driver->thisif);
    }
#endif
}

void AP_Networking_ChibiOS::link_down_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_ChibiOS *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_stop(driver->thisif);
    }
#endif
}


#if AP_NETWORKING_CAPTURE_ENABLED
/*
  capture all data in a pbuf chain
 */
void AP_Networking_ChibiOS::capture_pbuf(struct pbuf *p)
{
    auto *front = AP_Networking::singleton;
    if (!front->option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        return;
    }
    auto &driver = *(AP_Networking_ChibiOS*)front->backend;
    WITH_SEMAPHORE(driver.capture.sem);
    if (driver.capture.fd == -1) {
        return;
    }
    driver.capture_header(driver.capture.fd, p->tot_len);
    auto &fs = AP::FS();
    for (auto *pp = p; pp != nullptr; pp = pp->next) {
        fs.write(driver.capture.fd, (const uint8_t *)pp->payload, pp->len);
    }
}

/*
  capture a contiguous frame buffer
 */
void AP_Networking_ChibiOS::capture_frame(const uint8_t *buf, size_t len)
{
    auto *front = AP_Networking::singleton;
    if (!front->option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        return;
    }
    auto &driver = *(AP_Networking_ChibiOS*)front->backend;
    WITH_SEMAPHORE(driver.capture.sem);
    if (driver.capture.fd == -1) {
        return;
    }
    driver.capture_header(driver.capture.fd, len);
    AP::FS().write(driver.capture.fd, buf, len);
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

/*
 * This function does the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
int8_t AP_Networking_ChibiOS::low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;

#if AP_NETWORKING_CAPTURE_ENABLED
    capture_pbuf(p);
#endif

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    // Original path: transmit directly via MAC
    struct pbuf *q;
    MACTransmitDescriptor td;

    if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
        return ERR_TIMEOUT;
    }

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);        /* drop the padding word */
#endif

    /* Iterates through the pbuf chain. */
    for(q = p; q != NULL; q = q->next) {
        macWriteTransmitDescriptor(&td, (uint8_t *)q->payload, (size_t)q->len);
    }
    macReleaseTransmitDescriptorX(&td);

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);         /* reclaim the padding word */
#endif

    return ERR_OK;

#else // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    // Hub path: copy pbuf chain into contiguous frame buffer and send via lwIP port or switch
    static uint8_t framebuf[1522];
    size_t ofs = 0;
#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);        /* drop the padding word */
#endif
    for (struct pbuf *q = p; q != nullptr; q = q->next) {
        if (ofs + q->len > sizeof(framebuf)) {
            // too large
#if ETH_PAD_SIZE
            pbuf_header(p, ETH_PAD_SIZE);         /* reclaim the padding word */
#endif
            return ERR_MEM;
        }
        memcpy(&framebuf[ofs], q->payload, q->len);
        ofs += q->len;
    }
#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);         /* reclaim the padding word */
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
    if (s_switch_tx_cb != nullptr) {
        return s_switch_tx_cb(framebuf, ofs) ? ERR_OK : ERR_IF;
    }
    if (AP::network().port_lwip != nullptr) {
        AP::network().port_lwip->send_frame(framebuf, ofs);
        return ERR_OK;
    }
#endif
    return ERR_IF;
#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
}

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
/*
 * Receives a frame.
 * Allocates a pbuf and transfers the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
bool AP_Networking_ChibiOS::low_level_input(struct netif *netif, struct pbuf **pbuf)
{
    MACReceiveDescriptor rd;
    struct pbuf *q;
    u16_t len;

    (void)netif;

    if (macWaitReceiveDescriptor(&ETHD1, &rd, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }

    len = (u16_t)rd.size;

#if ETH_PAD_SIZE
    len += ETH_PAD_SIZE;        /* allow room for Ethernet padding */
#endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    *pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (*pbuf != nullptr) {
#if ETH_PAD_SIZE
        pbuf_header(*pbuf, -ETH_PAD_SIZE); /* drop the padding word */
#endif

        /* Iterates through the pbuf chain. */
        for(q = *pbuf; q != NULL; q = q->next) {
            macReadReceiveDescriptor(&rd, (uint8_t *)q->payload, (size_t)q->len);
        }
        macReleaseReceiveDescriptorX(&rd);

#if ETH_PAD_SIZE
        pbuf_header(*pbuf, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

#if AP_NETWORKING_CAPTURE_ENABLED
        capture_pbuf(*pbuf);
#endif

    } else {
        macReleaseReceiveDescriptorX(&rd);     // Drop packet
    }
  
    return true;
}
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

int8_t AP_Networking_ChibiOS::ethernetif_init(struct netif *netif)
{
    netif->state = NULL;
    netif->name[0] = 'm';
    netif->name[1] = 's';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* maximum transfer unit */
    netif->mtu = LWIP_NETIF_MTU;

    /* device capabilities */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

#if LWIP_IGMP
    // also enable multicast
    netif->flags |= NETIF_FLAG_IGMP;
#endif

    return ERR_OK;
}

#if AP_NETWORKING_CAPTURE_ENABLED
// start a pcap network capture
void AP_Networking_ChibiOS::start_capture(void)
{
    if (capture.fd != -1) {
        // called at 1Hz, flush the file
        AP::FS().fsync(capture.fd);
        return;
    }
    const struct pcap_hdr {
        uint32_t magic_number;   // 0xa1b2c3d4
        uint16_t version_major;  // 2
        uint16_t version_minor;  // 4
        int32_t  thiszone;       // GMT to local correction
        uint32_t sigfigs;        // accuracy of timestamps
        uint32_t snaplen;        // max length of captured packets, in octets
        uint32_t network;        // data link type (1 for Ethernet)
    } hdr = {
        0xa1b2c3d4, 2, 4, 0, 0, 1500, 1
    };
    const char *fname = "eth0.cap";
    WITH_SEMAPHORE(capture.sem);
    auto &fs = AP::FS();
    capture.fd = fs.open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    if (capture.fd != -1) {
        fs.write(capture.fd, (const void *)&hdr, sizeof(hdr));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Capturing to %s", fname);
    }
}

// stop a pcap network capture
void AP_Networking_ChibiOS::stop_capture(void)
{
    int fd = capture.fd;
    if (fd != -1) {
        capture.fd = -1;
        AP::FS().close(fd);
    }
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

/*
  networking thread
*/
void AP_Networking_ChibiOS::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    /* start tcpip thread */
    tcpip_init(NULL, NULL);

#if AP_NETWORKING_CONTROLS_HOST_MAC_SETTINGS_ENABLED
    frontend.param.macaddr.get_address(thisif->hwaddr);
#endif

    struct {
        ip4_addr_t ip, gateway, netmask;
    } addr {};

    if (!frontend.get_dhcp_enabled()) {
        addr.ip.addr = htonl(frontend.get_ip_param());
        addr.gateway.addr = htonl(frontend.get_gateway_param());
        addr.netmask.addr = htonl(frontend.get_netmask_param());
    }

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    // Original path: start MAC from ChibiOS backend
    const MACConfig mac_config = {thisif->hwaddr};
    macStart(&ETHD1, &mac_config);
#endif

    /* Add interface. */
    auto result = netifapi_netif_add(thisif, &addr.ip, &addr.netmask, &addr.gateway, NULL, ethernetif_init, tcpip_input);
    if (result != ERR_OK) {
        AP_HAL::panic("Failed to initialise netif");
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

#ifdef NEEDS_KSZ9896C_ERRATA
    apply_errata_for_mac_KSZ9896C();
#endif

    /* Setup event sources.*/
    event_timer_t evt;
    event_listener_t el0;
    
    evtObjectInit(&evt, LWIP_LINK_POLL_INTERVAL);
    evtStart(&evt);
    chEvtRegisterMask(&evt.et_es, &el0, PERIODIC_TIMER_ID);

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    // Original path: register for MAC RX events
    event_listener_t el1;
    chEvtRegisterMaskWithFlags(macGetEventSource(&ETHD1), &el1,
                               FRAME_RECEIVED_ID, MAC_FLAGS_RX);
    chEvtAddEvents(PERIODIC_TIMER_ID | FRAME_RECEIVED_ID);
#else
    // Hub path: register for lwIP port and COBS events
#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
    event_listener_t el2;
    if (AP::network().port_lwip != nullptr) {
        chEvtRegisterMask(AP::network().port_lwip->get_event_source(), &el2, LWIP_PORT_FRAME_ID);
    }
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
    event_timer_t evt_fast;
    event_listener_t el_fast;
    evtObjectInit(&evt_fast, FAST_COBS_POLL_INTERVAL);
    evtStart(&evt_fast);
    chEvtRegisterMask(&evt_fast.et_es, &el_fast, FAST_COBS_TIMER_ID);
#endif
    chEvtAddEvents(PERIODIC_TIMER_ID
#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
                   | LWIP_PORT_FRAME_ID
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
                   | FAST_COBS_TIMER_ID
#endif
                   );
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

    while (true) {
#if AP_NETWORKING_CAPTURE_ENABLED
        eventmask_t mask = chEvtWaitAnyTimeout(ALL_EVENTS, chTimeMS2I(1000));
#else
        eventmask_t mask = chEvtWaitAny(ALL_EVENTS);
#endif

        if (mask & PERIODIC_TIMER_ID) {
#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
            // Original path: poll MAC link status directly
            bool current_link_status = macPollLinkStatus(&ETHD1);
#else
            // Hub path: lwIP is considered link-up when at least 2 hub ports are up.
            uint8_t up_count = 0;
#if AP_NETWORKING_BACKEND_HUB
            if (AP::network().get_hub() != nullptr) {
                up_count = AP::network().get_hub()->get_num_ports_link_up();
            }
#endif
            const bool current_link_status = (up_count >= 2);
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

            if (current_link_status != netif_is_link_up(thisif)) {
                if (current_link_status) {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_up, thisif, 0);
                    tcpip_callback_with_block(link_up_cb, this, 0);
                }
                else {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_down, thisif, 0);
                    tcpip_callback_with_block(link_down_cb, this, 0);
                }
            }

#if AP_NETWORKING_CAPTURE_ENABLED
            if (frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
                start_capture();
            } else {
                stop_capture();
            }
#endif
        }

#if !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
        // Original path: receive frames directly from MAC
        if (mask & FRAME_RECEIVED_ID) {
            struct pbuf *p;
            while (low_level_input(thisif, &p)) {
                if (p != NULL) {
                    struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
                    switch (htons(ethhdr->type)) {
                        /* IP or ARP packet? */
                    case ETHTYPE_IP:
                    case ETHTYPE_ARP:
                        /* full packet send to tcpip_thread to process */
                        if (thisif->input(p, thisif) == ERR_OK) {
                            break;
                        }
                        /* Falls through */
                    default:
                        pbuf_free(p);
                    }
                }
            }
        }
#else
        // Hub path: receive frames from lwIP port or switch interface
#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
        static uint8_t rx_frame_buf[1522];
        if (mask & LWIP_PORT_FRAME_ID) {
            size_t len = 0;
            // Prefer switch RX callback if registered; otherwise use lwIP port queue
            if (s_switch_rx_cb != nullptr) {
                while (s_switch_rx_cb(rx_frame_buf, &len, sizeof(rx_frame_buf))) {
                    process_frame_to_lwip(rx_frame_buf, len, thisif);
                }
            } else {
                while (AP::network().port_lwip != nullptr &&
                       AP::network().port_lwip->get_frame(rx_frame_buf, &len, sizeof(rx_frame_buf))) {
                    process_frame_to_lwip(rx_frame_buf, len, thisif);
                }
            }
        }
#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
        if (mask & FAST_COBS_TIMER_ID) {
            const uint8_t n_cobs = AP::network().get_num_cobs_ports();
            for (uint8_t i = 0; i < n_cobs; i++) {
                auto *p = AP::network().get_cobs_port(i);
                if (p != nullptr) {
                    p->update();
                }
            }

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP
            // After processing COBS, check if lwIP port has frames queued
            if (!(mask & LWIP_PORT_FRAME_ID) && AP::network().port_lwip != nullptr) {
                size_t len = 0;
                while ((s_switch_rx_cb != nullptr && s_switch_rx_cb(rx_frame_buf, &len, sizeof(rx_frame_buf))) ||
                       (s_switch_rx_cb == nullptr && AP::network().port_lwip->get_frame(rx_frame_buf, &len, sizeof(rx_frame_buf)))) {
                    process_frame_to_lwip(rx_frame_buf, len, thisif);
                }
            }
#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP
        }
#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS
#endif // !AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
    }
}

#ifdef NEEDS_KSZ9896C_ERRATA
void AP_Networking_ChibiOS::apply_errata_for_mac_KSZ9896C()
{
#if AP_MODPAYLOAD_ENABLED
    if (AP::mod_payload() != nullptr && AP::mod_payload()->enabled() && AP::mod_payload()->board_version() != 0) {
        return;
    }
#endif

    /// Apply Erratas
    for (uint8_t phyaddr = 1; phyaddr <= 5; phyaddr++) {

        //ETHD1.phyaddr = phyaddr << 11;
        ETHD1.phyaddr = phyaddr << ETH_MACMDIOAR_PA_Pos;

        // Hardware Design Checklist
        // https://ww1.microchip.com/downloads/en/DeviceDoc/KSZ989x-KSZ956x-KSZ9477-Hardware-Design-Checklist-00004151.pdf
        // 6.4: 10/100 Mbps Ethernet Only
        mii_write(&ETHD1, 0x00, 0x3100); // disable 1000Gbps, enable auto-negotiate for 10/100
        mii_write(&ETHD1, 0x09, 0x0400); // disable 1000Gbps announcements
        mii_write(&ETHD1, 0x00, 0x3100 | (1 << 9)); // restart auto-negotiate

        // Erratas:
        // http://ww1.microchip.com/downloads/en/DeviceDoc/80000757A.pdf
        const uint16_t mmd[22][3] = {
            //[MMD], [register],[data]

            // module 1: Register settings are needed to improve PHY receive performance
            {0x01, 0x6F, 0xDD0B},
            {0x01, 0x8F, 0x6032},
            {0x01, 0x9D, 0x248C},
            {0x01, 0x75, 0x0060},
            {0x01, 0xD3, 0x7777},
            {0x1C, 0x06, 0x3008},
            {0x1C, 0x08, 0x2001},

            // module 2: Transmit waveform amplitude can be improved
            {0x1C, 0x04F, 0x00D0},

            // module 3: Energy Efficient Ethernet (EEE) feature select must be manually disabled
            {0x07, 0x03C, 0x0000},

            // module 4: Toggling PHY Powerdown can cause errors or link failures in adjacent PHYs
            #if STM32_MAC_ETH1_CHANGE_PHY_STATE
            #error "MII_KSZ9896C_ID Errata module 4 requires STM32_MAC_ETH1_CHANGE_PHY_STATE = FALSE"
            #endif

            // module 6: Register settings are required to meet data sheet supply current specifications
            {0x1C, 0x013, 0x6EFF}, // This particular Register is critical for an unknown reason.
            {0x1C, 0x014, 0xE6FF},
            {0x1C, 0x015, 0x6EFF},
            {0x1C, 0x016, 0xE6FF},
            {0x1C, 0x017, 0x00FF},
            {0x1C, 0x018, 0x43FF},
            {0x1C, 0x019, 0xC3FF},
            {0x1C, 0x01A, 0x6FFF},
            {0x1C, 0x01B, 0x07FF},
            {0x1C, 0x01C, 0x0FFF},
            {0x1C, 0x01D, 0xE7FF},
            {0x1C, 0x01E, 0xEFFF},
            {0x1C, 0x020, 0xEEEE},
        };

        
        for (uint8_t i=0; i<22; i++) {
            // Write MMD - Device Address 2h, Register 00h = 0010h to enable single-LED mode.
            // 1. Write the PHY MMD Setup Register with 0002h // Set up register address for MMD – Device Address 2h.
            // 2. Write the PHY MMD Data Register with 0000h // Select Register 00h of MMD – Device Address 2h.
            // 3. Write the PHY MMD Setup Register with 4002h // Select register data for MMD – Device Address 2h, Reg. 00h.
            // 4. Write the PHY MMD Data Register with 0010h // Write value 0010h to MMD – Device Address 2h, Reg. 00h.
            
            const uint16_t deviceAddress = (mmd[i][0] & 0x001F);
            mii_write(&ETHD1, 0x0D, 0x0000 | deviceAddress);
            mii_write(&ETHD1, 0x0E, mmd[i][1]);

            mii_write(&ETHD1, 0x0D, 0x4000 | deviceAddress);
            mii_write(&ETHD1, 0x0E, mmd[i][2]);
        }
    }
    ETHD1.phyaddr = BOARD_PHY_ADDRESS;
}
#endif // NEEDS_KSZ9896C_ERRATA

/*
  update called at 10Hz
*/
void AP_Networking_ChibiOS::update()
{
    const uint32_t ip = ntohl(thisif->ip_addr.addr);
    const uint32_t nm = ntohl(thisif->netmask.addr);
    const uint32_t gw = ntohl(thisif->gw.addr);

    if (ip != activeSettings.ip ||
        nm != activeSettings.nm ||
        gw != activeSettings.gw) {
        activeSettings.ip = ip;
        activeSettings.gw = gw;
        activeSettings.nm = nm;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
}

#endif // AP_NETWORKING_BACKEND_CHIBIOS
