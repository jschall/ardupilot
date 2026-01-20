#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#include "AP_Networking_Port_lwIP.h"

AP_Networking_Port_lwIP::AP_Networking_Port_lwIP(AP_Networking_Hub *hub_in) :
    hub(hub_in)
{
    chEvtObjectInit(&frame_available_event);
}

AP_Networking_Port_lwIP::~AP_Networking_Port_lwIP()
{
    if (rx_queue != nullptr) {
        delete rx_queue;
        rx_queue = nullptr;
    }
}

bool AP_Networking_Port_lwIP::init()
{
    if (rx_queue == nullptr) {
        rx_queue = NEW_NOTHROW ByteBuffer(RX_QUEUE_SIZE);
        if (rx_queue == nullptr) {
            return false;
        }
    }
    return true;
}

void AP_Networking_Port_lwIP::update()
{
    // nothing to do; event-driven
}

void AP_Networking_Port_lwIP::signal_frame_available()
{
    chEvtBroadcast(&frame_available_event);
}

void AP_Networking_Port_lwIP::deliver_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        rx_errors++;
        return;
    }
    WITH_SEMAPHORE(rx_sem);
    // 2-byte length header (little-endian) + payload
    if (rx_queue->space() < (uint32_t)(2 + len)) {
        // drop if queue full
        rx_errors++;
        return;
    }
    uint16_t l = (uint16_t)len;
    const uint8_t hdr[2] = { (uint8_t)(l & 0xFF), (uint8_t)((l >> 8) & 0xFF) };
    (void)rx_queue->write(hdr, 2);
    (void)rx_queue->write(frame, (uint32_t)len);
    rx_count++;
    signal_frame_available();
}

bool AP_Networking_Port_lwIP::can_receive() const
{
    // heuristic: enough space for at least one max-sized frame + header
    // Note: space() is a quick read of internal state; slight race is acceptable
    return rx_queue != nullptr && rx_queue->space() >= (2 + MAX_FRAME);
}

bool AP_Networking_Port_lwIP::get_frame(uint8_t *buf, size_t *len, size_t max_len)
{
    if (rx_queue == nullptr || buf == nullptr || len == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(rx_sem);
    if (rx_queue->available() < 2) {
        return false;
    }
    uint8_t hdr[2];
    if (rx_queue->peekbytes(hdr, 2) != 2) {
        return false;
    }
    const uint16_t l = (uint16_t)(hdr[0] | (uint16_t(hdr[1]) << 8));
    if (l == 0 || l > MAX_FRAME) {
        // corrupt length - reset queue
        rx_queue->clear();
        return false;
    }
    if (rx_queue->available() < (uint32_t)(2 + l)) {
        return false;
    }
    if (l > max_len) {
        // consumer-provided buffer too small, drop frame
        // discard this frame to allow progress
        (void)rx_queue->advance(2 + l);
        return false;
    }
    (void)rx_queue->advance(2);
    if (rx_queue->read(buf, l) != l) {
        return false;
    }
    *len = l;
    return true;
}

void AP_Networking_Port_lwIP::send_frame(const uint8_t *frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        tx_errors++;
        return;
    }
    hub->route_frame(this, frame, len);
    tx_count++;
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP

