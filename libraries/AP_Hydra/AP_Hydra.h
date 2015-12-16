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
#include <SITL/SITL.h>

#define HYDRA_MAX_PAYLOAD_LEN 10
#define HYDRA_OVERHEAD_SIZE 5

#define HYDRA_MSG_SET_TORQUE 0x01
#define HYDRA_MSG_ANG_POS 0x02

class AP_Hydra
{
public:
    void set_torque(int16_t torque_out) { _torque_out = torque_out; }
    float get_rotor_pos_rad() { return _rotor_pos_rad; }
    uint32_t get_rotor_pos_update_us() { return _rotor_pos_update_us; }
    virtual void update() = 0;
protected:
    int16_t  _torque_out = 0;
    float    _rotor_pos_rad = 0;
    uint32_t _rotor_pos_update_us = 0;
};

class AP_Hydra_SITL : public AP_Hydra
{
public:
    AP_Hydra_SITL(uint8_t instance, SITL::SITL& sitl) :
    _instance(instance),
    _sitl(sitl) {}
    virtual void update();
private:
    uint8_t _instance;
    SITL::SITL &_sitl;
};

class AP_Hydra_UART : public AP_Hydra
{
public:
    AP_Hydra_UART(uint8_t instance, const AP_SerialManager& serial_manager) :
    _instance(instance),
    _serial_manager(serial_manager) {}
    virtual void update();

private:
    struct hydra_msg_t {
        uint8_t msg_id;
        uint8_t payload_len;
        uint8_t payload[HYDRA_MAX_PAYLOAD_LEN];
    };

    void read();
    bool write(uint8_t msg_id, uint8_t payload_len, uint8_t* payload);
    bool parse_stream(hydra_msg_t& ret);
    void process_msg(const hydra_msg_t& ret);

    AP_Buffer<uint8_t, HYDRA_MAX_PAYLOAD_LEN+HYDRA_OVERHEAD_SIZE> _buf;

    const AP_SerialManager& _serial_manager;
    AP_HAL::UARTDriver *_port;
    uint8_t _instance;

    int16_t    _torque_out;
    float _rotor_position_rad;
    uint32_t _pos_update_us;
};

#endif // __AP_HYDRA_H__
