#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS

#include "AP_Networking_Port_COBS.h"
#include "AP_Networking_Hub.h"
#include "AP_Networking_Port_Ethernet.h"
#include "AP_Networking.h"
#include <AP_Math/crc.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// COBS keepalive marker payload
static const uint8_t COBS_KA_PAYLOAD[] = {'K','A','L','I','V','E',0x01};
static constexpr size_t COBS_KA_LEN = sizeof(COBS_KA_PAYLOAD);

AP_Networking_Port_COBS::AP_Networking_Port_COBS(AP_Networking_Hub *hub_in,
        AP_HAL::UARTDriver *uart_in,
        uint32_t baud_rate) :
    hub(hub_in),
    uart(uart_in),
    uart_baud(baud_rate)
{
}

AP_Networking_Port_COBS::~AP_Networking_Port_COBS()
{
    if (tx_buffer != nullptr) {
        delete tx_buffer;
        tx_buffer = nullptr;
    }
}

bool AP_Networking_Port_COBS::init()
{
    if (hub == nullptr || uart == nullptr) {
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    last_rx_ms = last_tx_ms = last_keepalive_ms = now;
    if (tx_buffer == nullptr) {
        // fixed 64 KiB transmit ring
        static constexpr uint32_t TX_RING_SIZE = 64U * 1024U;
        tx_buffer = NEW_NOTHROW ByteBuffer(TX_RING_SIZE);
        if (tx_buffer == nullptr) {
            return false;
        }
    }
    // Ensure UART is open at desired baud; set TX buffer to 64KiB (uint16_t capped)
    uart->begin(uart_baud, 2048, 65535);
    return true;
}

void AP_Networking_Port_COBS::process_rx()
{
    // Limit time to avoid watchdogs
    const uint32_t start_us = AP_HAL::micros();
    while (uart->available() > 0) {
        uint8_t byte = 0;
        const auto nread = uart->read(&byte, 1);
        if (nread != 1) {
            break;
        }
        uart_rx_bytes += (uint32_t)nread;
        // Feed streaming decoder
        if (cobs_decoder.process_byte(byte)) {
            // A frame is complete
            size_t frame_len = sizeof(rx_frame_buffer);
            if (!cobs_decoder.get_frame(rx_frame_buffer, &frame_len, sizeof(rx_frame_buffer))) {
                cobs_decode_errors++;
                cobs_decoder.reset();  // Reset decoder on error
                continue;
            }
            // Reset decoder for next frame (after successfully getting frame)
            cobs_decoder.reset();
            (void)handle_decoded_frame(frame_len);
        } else if (cobs_decoder.get_state() == AP_Networking_COBS::Decoder::State::ERROR) {
            // recover promptly from decoder error to re-sync on next delimiter
            cobs_decode_errors++;
            cobs_decoder.reset();
            continue;
        } else {
            // not a complete frame yet
            continue;
        }
        // Time bound (~1ms)
        if ((AP_HAL::micros() - start_us) > 1000U) {
            break;
        }
    }

    // After processing all available bytes, check if a frame ended at a code block boundary
    // This handles frames that complete without requiring the next frame's delimiter
    if (cobs_decoder.process_end_of_input()) {
        size_t frame_len = sizeof(rx_frame_buffer);
        if (cobs_decoder.get_frame(rx_frame_buffer, &frame_len, sizeof(rx_frame_buffer))) {
            cobs_decoder.reset();
            (void)handle_decoded_frame(frame_len);
        } else {
            cobs_decode_errors++;
            cobs_decoder.reset();
        }
    }
}

void AP_Networking_Port_COBS::drain_tx()
{
    while (true) {
        if (!tx_sem.take(1)) {
            break;
        }
        if (tx_buffer->is_empty()) {
            tx_sem.give();
            break;
        }
        uint32_t avail = 0;
        const uint8_t *ptr = tx_buffer->readptr(avail);
        if (ptr == nullptr || avail == 0) {
            tx_sem.give();
            break;
        }
        const auto n = uart->write(ptr, avail);
        if (n == 0) {
            tx_sem.give();
            uart_tx_stalls++;
            break;
        }
        (void)tx_buffer->advance(n);
        tx_sem.give();
        uart_tx_bytes += (uint32_t)n;
    }
}

void AP_Networking_Port_COBS::update()
{
    process_rx();
    // Send periodic keepalive if idle (no data TX recently)
    const uint32_t now = AP_HAL::millis();
    if ((now - last_tx_ms) >= KEEPALIVE_INTERVAL_MS &&
        (now - last_keepalive_ms) >= KEEPALIVE_INTERVAL_MS) {
        send_keepalive();
        last_keepalive_ms = now;
    }
    drain_tx();
}

void AP_Networking_Port_COBS::deliver_frame(const uint8_t *frame, size_t len)
{
    // Validate bounds (we will pad frames shorter than MIN_FRAME_NOFCS)
    if (frame == nullptr || len == 0 || len > MAX_FRAME) {
        tx_errors++;
        return;
    }

    // Serialize use of tx_encode_buffer and writes to tx_buffer
    if (!tx_sem.take(1)) {
        tx_dropped++;
        return;
    }

    // Build contiguous [frame + CRC] in tx_input_buffer
    const size_t in_len = len + 4;
    if (in_len > sizeof(tx_input_buffer)) {
        // Should not happen; drop
        tx_errors++;
        tx_sem.give();
        return;
    }
    memcpy(tx_input_buffer, frame, len);
    const uint32_t crc = crc_crc32(0, frame, len);
    memcpy(&tx_input_buffer[len], &crc, sizeof(crc)); // LE

    // COBS encode into tx_encode_buffer
    // max COBS expansion is input + input/254 + 2; ensure capacity
    const size_t max_encoded = in_len + (in_len / 254U) + 2U;
    if (max_encoded > sizeof(tx_encode_buffer)) {
        tx_errors++;
        tx_sem.give();
        return;
    }

    const size_t enc_len = AP_Networking_COBS::encode(tx_input_buffer, in_len, tx_encode_buffer, sizeof(tx_encode_buffer));
    if (enc_len == 0) {
        tx_errors++;
        tx_sem.give();
        return;
    }

    // Append zero delimiter
    if (tx_buffer->space() < enc_len + 1) {
        tx_dropped++;
        tx_sem.give();
        return;
    }
    // We already hold tx_sem; perform the write
    (void)tx_buffer->write(tx_encode_buffer, enc_len);
    const uint8_t zero = 0;
    (void)tx_buffer->write(&zero, 1);
    tx_sem.give();
    tx_count++;
    last_tx_ms = AP_HAL::millis();
}

bool AP_Networking_Port_COBS::can_receive() const
{
    // Always accept if we have space in TX buffer for encoded frame
    // Worst-case encoded length for MAX_FRAME+CRC plus delimiter
    const size_t worst = (MAX_FRAME + 4) + ((MAX_FRAME + 4) / 254U) + 3U;
    return tx_buffer != nullptr && tx_buffer->space() >= worst;
}

void AP_Networking_Port_COBS::send_keepalive()
{
    // small payload (<64) so it won't be routed as Ethernet; still CRC-protected
    const size_t ka_len = COBS_KA_LEN;
    // Serialize use of tx_encode_buffer and writes to tx_buffer
    if (!tx_sem.take(1)) {
        return;
    }
    // Build into tx_encode_buffer
    if (ka_len + 4 > sizeof(tx_encode_buffer)) {
        tx_sem.give();
        return;
    }
    memcpy(tx_encode_buffer, COBS_KA_PAYLOAD, ka_len);
    const uint32_t crc = crc_crc32(0, COBS_KA_PAYLOAD, ka_len);
    memcpy(&tx_encode_buffer[ka_len], &crc, sizeof(crc));
    const size_t in_len = ka_len + 4;
    const size_t max_encoded = in_len + (in_len / 254U) + 2U;
    if (max_encoded > sizeof(tx_encode_buffer)) {
        tx_sem.give();
        return;
    }
    // Encode into the tail of tx_encode_buffer to avoid overlap with input
    uint8_t *encoded = &tx_encode_buffer[sizeof(tx_encode_buffer) - max_encoded];
    const size_t enc_len = AP_Networking_COBS::encode(tx_encode_buffer, in_len, encoded, max_encoded);
    if (enc_len == 0) {
        tx_sem.give();
        return;
    }
    // Append zero delimiter
    if (tx_buffer->space() < enc_len + 1) {
        tx_sem.give();
        return;
    }
    // We already hold tx_sem; perform the write
    (void)tx_buffer->write(encoded, enc_len);
    const uint8_t zero = 0;
    (void)tx_buffer->write(&zero, 1);
    tx_sem.give();
    // don't count as tx_count (to keep stats focused on Ethernet frames)
    ka_tx_count++;
}

bool AP_Networking_Port_COBS::is_link_up() const
{
    // Treat link as up if we've seen any RX (frame or keepalive) in the last 2 seconds
    const uint32_t now = AP_HAL::millis();
    return (now - last_rx_ms) <= (KEEPALIVE_INTERVAL_MS * 4U);
}

bool AP_Networking_Port_COBS::handle_decoded_frame(size_t frame_len)
{
    if (frame_len < 4) {
        rx_errors++;
        return false;
    }
    const size_t data_len = frame_len - 4;
    uint32_t rx_crc = 0;
    memcpy(&rx_crc, &rx_frame_buffer[data_len], 4);
    const uint32_t calc_crc = crc_crc32(0, rx_frame_buffer, data_len);
    if (rx_crc != calc_crc) {
        crc_errors++;
        if ((crc_errors % 10) == 0) {
            last_crc_error.error_count = crc_errors;
            last_crc_error.data_len = data_len;
            last_crc_error.frame_len = frame_len;
            last_crc_error.rx_crc = rx_crc;
            last_crc_error.calc_crc = calc_crc;
            const size_t copy_len = (data_len < ARRAY_SIZE(last_crc_error.data)) ? data_len : ARRAY_SIZE(last_crc_error.data);
            memcpy(last_crc_error.data, rx_frame_buffer, copy_len);
            if (copy_len < ARRAY_SIZE(last_crc_error.data)) {
                memset(&last_crc_error.data[copy_len], 0, ARRAY_SIZE(last_crc_error.data) - copy_len);
            }
            const size_t tail_start = (frame_len >= ARRAY_SIZE(last_crc_error.tail)) ? frame_len - ARRAY_SIZE(last_crc_error.tail) : 0;
            const size_t tail_len = frame_len - tail_start;
            memcpy(last_crc_error.tail, &rx_frame_buffer[tail_start], tail_len);
            if (tail_len < ARRAY_SIZE(last_crc_error.tail)) {
                memset(&last_crc_error.tail[tail_len], 0, ARRAY_SIZE(last_crc_error.tail) - tail_len);
            }
            last_crc_error.has_error = true;
        }
        return false;
    }
    if (frame_len > max_successful_frame_len) {
        max_successful_frame_len = frame_len;
    }
    if (data_len == COBS_KA_LEN && memcmp(rx_frame_buffer, COBS_KA_PAYLOAD, COBS_KA_LEN) == 0) {
        last_rx_ms = AP_HAL::millis();
        ka_rx_count++;
    } else if (data_len > 0 && data_len <= MAX_FRAME) {
        hub->route_frame(this, rx_frame_buffer, data_len);
        rx_count++;
        last_rx_ms = AP_HAL::millis();
    }
    return true;
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS
