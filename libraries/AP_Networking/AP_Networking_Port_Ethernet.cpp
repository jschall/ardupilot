#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Port_Ethernet.h"
#include "AP_Networking_ChibiOS.h"

AP_Networking_Port_Ethernet *AP_Networking_Port_Ethernet::singleton = nullptr;

bool AP_Networking_Port_Ethernet::init()
{
    if (singleton != nullptr) {
        return false; // already initialized
    }
    singleton = this;

    // Register RX hook - we handle MAC received frames
    AP_Networking_ChibiOS::set_rx_hook(rx_hook);

    return true;
}

void AP_Networking_Port_Ethernet::rx_hook(const uint8_t *frame, size_t len)
{
    if (singleton == nullptr || singleton->hub == nullptr) {
        return;
    }
    singleton->rx_count++;
    // Route MAC-received frame through hub (to COBS, lwIP, etc.)
    singleton->hub->route_frame(singleton, frame, len);
}

void AP_Networking_Port_Ethernet::deliver_frame(const uint8_t *frame, size_t len)
{
    // Hub wants to send a frame out via Ethernet MAC
    if (AP_Networking_ChibiOS::send_frame_to_mac(frame, len)) {
        tx_count++;
    } else {
        tx_errors++;
    }
}

bool AP_Networking_Port_Ethernet::is_link_up() const
{
    return AP_Networking_ChibiOS::get_link_status();
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
