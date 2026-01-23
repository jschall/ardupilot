#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#include "AP_Networking_Hub.h"

/*
  lwIP port for hub.
  Hooks into ChibiOS backend's lwIP TX to route frames through hub.
  Receives frames from hub via inject_frame_to_lwip.
*/
class AP_Networking_Port_lwIP : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_lwIP(AP_Networking_Hub *hub_in) : hub(hub_in) {}

    CLASS_NO_COPY(AP_Networking_Port_lwIP);

    bool init();
    void update() override {}

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override { return true; }
    const char *get_name() const override { return "lwIP"; }
    bool is_link_up() const override { return true; }  // lwIP is always "up"

    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_tx_errors() const { return tx_errors; }

private:
    AP_Networking_Hub *hub;

    // TX hook callback - called by ChibiOS backend when lwIP sends a frame
    static void tx_hook(const uint8_t *frame, size_t len);

    static AP_Networking_Port_lwIP *singleton;

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP
