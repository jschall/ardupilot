#include "AP_Hydra.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

void AP_Hydra::init(const AP_SerialManager& serial_manager) {
    for (uint8_t i=0; i<HYDRA_MAX_INSTANCES; i++) {
        _port[i] = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hydra, i);
        if (_port[i] != NULL) {
            _enabled_mask |= 1<<i;
        }
    }
}

void AP_Hydra::read(uint8_t instance)
{
    if (!enabled(instance)) {
        return;
    }

    while (_port[instance]->available() > 0) {
        hydra_msg_t msg;

        while (!_buf[instance].is_full() && _port[instance]->available() > 0) {
            _buf[instance].push_back(_port[instance]->read());
        }

        if (check_for_msg(instance, msg)) {
            if (msg.msg_id == 0x02 && msg.payload_len == sizeof(uint16_t)) {
                uint16_t rotor_pos;
                memcpy(&rotor_pos, &msg.payload, sizeof(uint16_t));
                _rotor_position_rad[instance] = rotor_pos * (2.0f*M_PI)/65536.0f;
                _pos_update_us[instance] = AP_HAL::micros();
                hal.console->printf("%.0f\n", degrees(_rotor_position_rad[instance]));
            }
        }
    }
}

bool AP_Hydra::write(uint8_t instance, uint8_t msg_id, uint8_t payload_len, uint8_t* payload)
{
    if (!enabled(instance)) {
        return false;
    }

    if (_port[instance]->txspace() < payload_len + HYDRA_OVERHEAD_SIZE) {
        return false;
    }

    uint8_t header[] = {
        0x11,
        0x94,
        payload_len,
        msg_id
    };

    _port[instance]->write(header, sizeof(header));
    _port[instance]->write(payload, payload_len);
    uint16_t crc;
    crc = crc16_ccitt(header, sizeof(header), 0);
    crc = crc16_ccitt(payload, payload_len, crc);
    _port[instance]->write((uint8_t*)&crc, sizeof(crc));

    return true;
}

bool AP_Hydra::check_for_msg(uint8_t instance, AP_Hydra::hydra_msg_t& ret)
{
    if (!enabled(instance)) {
        return false;
    }
    // messages must start with 0x11 0x94
    while(true) {
        if (_buf[instance].size() > 1 && (_buf[instance].peek(0) != 0x11 || _buf[instance].peek(1) != 0x94)) {
            _buf[instance].pop_front();
        } else if (_buf[instance].size() > 0 && _buf[instance].peek(0) != 0x11) {
            _buf[instance].pop_front();
        } else {
            break;
        }
    }

    if (_buf[instance].size() < HYDRA_OVERHEAD_SIZE) {
        return false;
    }

    uint8_t payload_len = _buf[instance].peek(2);
    uint8_t msg_len = HYDRA_OVERHEAD_SIZE+payload_len;

    if (payload_len > HYDRA_MAX_PAYLOAD_LEN) {
        _buf[instance].clear();
        return false;
    }

    if (_buf[instance].size() < msg_len) {
        return false;
    }

    union {
        uint16_t uint16_val;
        uint8_t uint8_array[2];
    } msg_crc;

    msg_crc.uint16_val = 0;
    for (uint8_t i=0; i<msg_len-2; i++) {
        msg_crc.uint16_val = crc16_ccitt(&_buf[instance].peek(i), 1, msg_crc.uint16_val);
    }

    if (msg_crc.uint8_array[0] != _buf[instance].peek(msg_len-2) || msg_crc.uint8_array[1] != _buf[instance].peek(msg_len-1)) {
        // failed CRC - remove message from buffer
        for (uint8_t i=0; i<msg_len; i++) {
            _buf[instance].pop_front();
        }
        hal.console->printf("crc %u %u expected %u %u\n", msg_crc.uint8_array[0], msg_crc.uint8_array[1], _buf[instance].peek(msg_len-2), _buf[instance].peek(msg_len-1));
        return false;
    }

    // fill return value
    memset(&ret, 0, sizeof(ret));
    ret.msg_id = _buf[instance].peek(2);
    ret.payload_len = payload_len;
    for (uint8_t i=0; i<payload_len; i++) {
        ret.payload[i] = _buf[instance].peek(4+i);
    }

    // remove message from buffer
    for (uint8_t i=0; i<msg_len; i++) {
        _buf[instance].pop_front();
    }

    return true;
}

void AP_Hydra::set_torque(uint8_t instance, int16_t torque_out) {
    if (enabled(instance)) {
        _torque_out[instance] = torque_out;
        write(instance, 0x01, 2, (uint8_t*)&(_torque_out[instance]));
    }
}
