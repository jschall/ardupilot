#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET

#include "AP_Networking_Hub.h"
#include "AP_Networking.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>
#include <hal.h>

/*
  Ethernet MAC port for ChibiOS
*/
class AP_Networking_Port_Ethernet : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_Ethernet(AP_Networking &frontend_in, AP_Networking_Hub *hub_in, const uint8_t macaddr_in[6]) :
        frontend(frontend_in),
        hub(hub_in)
    {
        macaddr[0] = macaddr_in[0];
        macaddr[1] = macaddr_in[1];
        macaddr[2] = macaddr_in[2];
        macaddr[3] = macaddr_in[3];
        macaddr[4] = macaddr_in[4];
        macaddr[5] = macaddr_in[5];
    }

    CLASS_NO_COPY(AP_Networking_Port_Ethernet);

    bool init();
    void update() override;

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override
    {
        return "Ethernet";
    }
    bool is_link_up() const override;

    // Link status helper
    bool poll_link_status() const;

    // Statistics
    uint32_t get_rx_count() const
    {
        return rx_count;
    }
    uint32_t get_tx_count() const
    {
        return tx_count;
    }
    uint32_t get_rx_errors() const
    {
        return rx_errors;
    }
    uint32_t get_tx_errors() const
    {
        return tx_errors;
    }

private:
    AP_Networking &frontend;
    AP_Networking_Hub *hub;
    uint8_t macaddr[6];

    // initialize MAC and set promiscuous receive
    bool mac_init_and_start();
    void set_promiscuous_mode();

    // RX/TX thread functions
    void process_rx();
    void rx_thread();
    void tx_thread();

    // TX queue (single reader: tx_thread, multiple writers: deliver_frame)
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t TX_QUEUE_SIZE = 16 * (MAX_FRAME + 2);
    ByteBuffer *tx_queue = nullptr;
    HAL_Semaphore tx_mutex;          // guards writers
    HAL_BinarySemaphore tx_not_empty; // signals queued frames

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_ETHERNET


