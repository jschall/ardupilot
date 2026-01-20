#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_LWIP

#include "AP_Networking_Hub.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>
#include <hal.h>

/*
  lwIP port that bridges frames between lwIP and hub
*/
class AP_Networking_Port_lwIP : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_lwIP(AP_Networking_Hub *hub_in);
    ~AP_Networking_Port_lwIP();

    CLASS_NO_COPY(AP_Networking_Port_lwIP);

    bool init();
    void update() override;

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override
    {
        return "lwIP";
    }
    // lwIP virtual port is considered up while enabled/initialized
    bool is_link_up() const override
    {
        return true;
    }

    // Called by lwIP low_level_input
    bool get_frame(uint8_t *buf, size_t *len, size_t max_len);

    // Called by lwIP low_level_output
    void send_frame(const uint8_t *frame, size_t len);

    // Event source to signal frames are available to lwIP thread
    event_source_t *get_event_source()
    {
        return &frame_available_event;
    }

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
    AP_Networking_Hub *hub;

    // Frame queue for lwIP
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t RX_QUEUE_SIZE = 16 * MAX_FRAME + 32; // include space for length headers
    ByteBuffer *rx_queue;
    HAL_Semaphore rx_sem;

    // Event source for signaling frame availability to lwIP thread
    event_source_t frame_available_event;

    void signal_frame_available();

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_LWIP


