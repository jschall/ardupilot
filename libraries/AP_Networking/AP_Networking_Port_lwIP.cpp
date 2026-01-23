#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#include "AP_Networking_Port_lwIP.h"
#include "AP_Networking_ChibiOS.h"

AP_Networking_Port_lwIP *AP_Networking_Port_lwIP::singleton = nullptr;

bool AP_Networking_Port_lwIP::init()
{
    if (singleton != nullptr) {
        return false; // already initialized
    }
    singleton = this;

    // Register TX hook - we handle lwIP transmitted frames
    AP_Networking_ChibiOS::set_tx_hook(tx_hook);

    return true;
}

void AP_Networking_Port_lwIP::tx_hook(const uint8_t *frame, size_t len)
{
    if (singleton == nullptr || singleton->hub == nullptr) {
        return;
    }
    singleton->tx_count++;
    // Route lwIP-transmitted frame through hub (to COBS, Ethernet, etc.)
    // Note: frame is already going to MAC via ChibiOS backend, this routes to other ports
    singleton->hub->route_frame(singleton, frame, len);
}

void AP_Networking_Port_lwIP::deliver_frame(const uint8_t *frame, size_t len)
{
    // Hub wants to deliver a frame to lwIP (e.g., from COBS)
    if (frame == nullptr || len == 0) {
        rx_errors++;
        return;
    }
    AP_Networking_ChibiOS::inject_frame_to_lwip(frame, len);
    rx_count++;
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP
