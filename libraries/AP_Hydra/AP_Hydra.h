#ifndef __AP_HYDRA_H__
#define __AP_HYDRA_H__

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Buffer/AP_Buffer.h>

#define HYDRA_MAX_INSTANCES 2

#define HYDRA_MAX_PAYLOAD_LEN 10
#define HYDRA_OVERHEAD_SIZE 6

#define HYDRA_MSG_SET_TORQUE 0x01
#define HYDRA_MSG_ANG_POS 0x02

class AP_Hydra
{
public:
    void init(const AP_SerialManager& serial_manager);
    void set_torque(uint8_t instance, int16_t torque_out);
    bool enabled(uint8_t instance) { return (_enabled_mask>>instance) & 1; }
    void read(uint8_t instance);

private:
    struct hydra_msg_t {
        uint8_t msg_id;
        uint8_t payload_len;
        uint8_t payload[HYDRA_MAX_PAYLOAD_LEN];
    };

    bool write(uint8_t instance, uint8_t msg_id, uint8_t payload_len, uint8_t* payload);
    bool check_for_msg(uint8_t instance, hydra_msg_t& ret);

    uint8_t    _enabled_mask;
    int16_t    _torque_out[HYDRA_MAX_INSTANCES];
    float _rotor_position_rad[HYDRA_MAX_INSTANCES];
    uint32_t _pos_update_us[HYDRA_MAX_INSTANCES];

    AP_Buffer<uint8_t, HYDRA_MAX_PAYLOAD_LEN+HYDRA_OVERHEAD_SIZE> _buf[HYDRA_MAX_INSTANCES];
    AP_HAL::UARTDriver *_port[HYDRA_MAX_INSTANCES];
};

#endif // __AP_HYDRA_H__
