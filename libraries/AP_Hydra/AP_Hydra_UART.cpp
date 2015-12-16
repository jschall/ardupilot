#include "AP_Hydra.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

void AP_Hydra_UART::update()
{
    _port = _serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hydra, _instance);
    if (_port == NULL) {
        return;
    }

    // write torque out
    write(0x01, 2, (uint8_t*)&_torque_out);

    // read messages
    read();
}

void AP_Hydra_UART::process_msg(const hydra_msg_t& msg)
{
    switch(msg.msg_id) {
        case 0x02:
            if (msg.payload_len == sizeof(uint16_t)) {
                uint16_t rotor_pos;
                memcpy(&rotor_pos, &msg.payload, sizeof(uint16_t));
                _rotor_pos_rad = rotor_pos * (2.0f*M_PI)/65536.0f;
                _rotor_pos_update_us = AP_HAL::micros();
            }
            break;
        default:
            break;
    }
}

bool AP_Hydra_UART::write(uint8_t msg_id, uint8_t payload_len, uint8_t* payload)
{
    if (_port == NULL || _port->txspace() < payload_len + HYDRA_OVERHEAD_SIZE) {
        return false;
    }

    uint8_t header[] = {
        0x11,
        0x94,
        payload_len,
        msg_id
    };

    _port->write(header, sizeof(header));
    _port->write(payload, payload_len);
    uint16_t crc;
    crc = crc16_ccitt(header, sizeof(header), 0);
    crc = crc16_ccitt(payload, payload_len, crc);
    _port->write((uint8_t*)&crc, sizeof(crc));

    return true;
}

void AP_Hydra_UART::read()
{
    if (_port == NULL) {
        return;
    }

    while (_port->available() > 0) {
        while (!_buf.is_full() && _port->available() > 0) {
            _buf.push_back(_port->read());
        }

        hydra_msg_t msg;
        if (parse_stream(msg)) {
            process_msg(msg);
        }
    }
}

bool AP_Hydra_UART::parse_stream(AP_Hydra_UART::hydra_msg_t& ret)
{
    // messages must start with 0x11 0x94
    while(true) {
        if (_buf.size() > 1 && (_buf.peek(0) != 0x11 || _buf.peek(1) != 0x94)) {
            _buf.pop_front();
        } else if (_buf.size() > 0 && _buf.peek(0) != 0x11) {
            _buf.pop_front();
        } else {
            break;
        }
    }

    if (_buf.size() < HYDRA_OVERHEAD_SIZE) {
        return false;
    }

    uint8_t payload_len = _buf.peek(2);
    uint8_t msg_len = HYDRA_OVERHEAD_SIZE+payload_len;

    if (payload_len > HYDRA_MAX_PAYLOAD_LEN) {
        _buf.clear();
        return false;
    }

    if (_buf.size() < msg_len) {
        return false;
    }

    union {
        uint16_t uint16_val;
        uint8_t uint8_array[2];
    } msg_crc;

    msg_crc.uint16_val = 0;
    for (uint8_t i=0; i<msg_len-2; i++) {
        msg_crc.uint16_val = crc16_ccitt(&_buf.peek(i), 1, msg_crc.uint16_val);
    }

    if (msg_crc.uint8_array[0] != _buf.peek(msg_len-2) || msg_crc.uint8_array[1] != _buf.peek(msg_len-1)) {
        // failed CRC - remove message from buffer
        for (uint8_t i=0; i<msg_len; i++) {
            _buf.pop_front();
        }
        hal.console->printf("crc %u %u expected %u %u\n", msg_crc.uint8_array[0], msg_crc.uint8_array[1], _buf.peek(msg_len-2), _buf.peek(msg_len-1));
        return false;
    }

    // fill return value
    memset(&ret, 0, sizeof(ret));
    ret.msg_id = _buf.peek(2);
    ret.payload_len = payload_len;
    for (uint8_t i=0; i<payload_len; i++) {
        ret.payload[i] = _buf.peek(4+i);
    }

    // remove message from buffer
    for (uint8_t i=0; i<msg_len; i++) {
        _buf.pop_front();
    }

    return true;
}
