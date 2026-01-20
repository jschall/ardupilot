#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS

#include "AP_Networking_Hub.h"
#include "AP_Networking_COBS.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>

/*
  UART <-> Hub port using COBS framing with CRC32
  - Frames from hub are appended with CRC32 (LE), COBS-encoded and written to UART
  - Frames from UART are COBS-decoded, CRC-validated and routed to hub
*/
class AP_Networking_Port_COBS : public AP_Networking_HubPort
{
public:
    AP_Networking_Port_COBS(AP_Networking_Hub *hub,
                            AP_HAL::UARTDriver *uart,
                            uint32_t baud_rate);
    ~AP_Networking_Port_COBS();

    CLASS_NO_COPY(AP_Networking_Port_COBS);

    bool init();
    void update() override; // time-bounded processing

    // AP_Networking_HubPort interface
    void deliver_frame(const uint8_t *frame, size_t len) override;
    bool can_receive() const override;
    const char *get_name() const override
    {
        return "COBS";
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
    uint32_t get_cobs_decode_errors() const
    {
        return cobs_decode_errors;
    }
    uint32_t get_crc_errors() const
    {
        return crc_errors;
    }
    uint32_t get_tx_dropped() const
    {
        return tx_dropped;
    }

    // Link heuristic: consider link up if we've received anything (KA or frame) recently
    bool is_link_up() const override;

private:
    AP_Networking_Hub *hub;
    AP_HAL::UARTDriver *uart;
    uint32_t uart_baud;

    // COBS decoder
    AP_Networking_COBS::Decoder cobs_decoder;

    // RX buffer (decoded frame + CRC)
    static constexpr size_t MAX_FRAME = 1522;
    // ChibiOS frame lengths exclude FCS; Ethernet minimum on-wire is 64 bytes => 60 bytes without FCS
    static constexpr size_t MIN_FRAME_NOFCS = 60;
    uint8_t rx_frame_buffer[MAX_FRAME + 4];

    // TX ring buffer for COBS-encoded frames
    ByteBuffer *tx_buffer;

    // per-instance COBS encode scratch (avoid large stack frames)
    uint8_t tx_encode_buffer[1600];
    // contiguous input buffer for [frame + CRC] to avoid overlap during encoding
    uint8_t tx_input_buffer[MAX_FRAME + 4];

    // protect multi-writer access to tx_buffer (deliver_frame + keepalive)
    HAL_Semaphore tx_sem;

    // keepalive state
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t last_keepalive_ms;
    static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500; // send KA while idle
    void send_keepalive();

    void process_rx();  // read UART, COBS-decode, route to hub
    void drain_tx();    // write from tx_buffer to UART

    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t rx_errors;
    uint32_t tx_errors;
    uint32_t cobs_decode_errors;
    uint32_t crc_errors;
    uint32_t tx_dropped;
    uint32_t ka_tx_count;
    uint32_t ka_rx_count;
    uint32_t uart_rx_bytes;
    uint32_t uart_tx_bytes;
    uint32_t uart_tx_stalls;
    size_t max_successful_frame_len;

    // Latest CRC error details (for debugging)
    struct CRCErrorDetails {
        uint32_t error_count;
        size_t data_len;
        size_t frame_len;  // Total decoded frame length
        uint32_t rx_crc;
        uint32_t calc_crc;
        uint8_t data[64];  // First 64 bytes of decoded frame
        uint8_t tail[8];   // Last 8 bytes of decoded frame (should contain CRC)
        bool has_error;
    } last_crc_error;

public:
    uint32_t get_keepalive_tx() const
    {
        return ka_tx_count;
    }
    uint32_t get_keepalive_rx() const
    {
        return ka_rx_count;
    }
    uint32_t get_uart_rx_bytes() const
    {
        return uart_rx_bytes;
    }
    uint32_t get_uart_tx_bytes() const
    {
        return uart_tx_bytes;
    }
    uint32_t get_uart_tx_stalls() const
    {
        return uart_tx_stalls;
    }
    size_t get_max_successful_frame_len() const
    {
        return max_successful_frame_len;
    }
    const CRCErrorDetails &get_last_crc_error() const
    {
        return last_crc_error;
    }
};

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS


