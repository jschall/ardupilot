#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Hub.h"

/*
  Ethernet MAC port for hub.
  Hooks into ChibiOS backend's MAC RX to route frames through hub.
*/
class AP_Networking_Port_Ethernet : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_Ethernet(AP_Networking_Hub *hub_in) : hub(hub_in) {}

    CLASS_NO_COPY(AP_Networking_Port_Ethernet);

    bool init();
    void update() override {}

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override { return true; }
    const char *get_name() const override { return "Ethernet"; }
    bool is_link_up() const override;

    // Statistics
    uint32_t get_rx_count() const { return rx_count; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_errors() const { return rx_errors; }
    uint32_t get_tx_errors() const { return tx_errors; }

private:
    AP_Networking_Hub *hub;

    // RX hook callback - called by ChibiOS backend when MAC receives a frame
    static void rx_hook(const uint8_t *frame, size_t len);

    static AP_Networking_Port_Ethernet *singleton;

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET
