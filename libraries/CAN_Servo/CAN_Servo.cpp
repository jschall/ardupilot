#include "CAN_Servo.h"

extern const AP_HAL::HAL& hal;

CAN_Servo::CAN_Servo(uint8_t bus, uint32_t angle_command_id, uint32_t angle_feedback_id) :
_angle_command_id(angle_command_id),
_angle_feedback_id(angle_feedback_id),
_bus(bus)
{
    hal.can->register_receiver(1<<bus, FUNCTOR_BIND_MEMBER(can_recv, void, uint32_t, const struct AP_HAL::CAN::message_s&));
}

void CAN_Servo::send_angle_target(float angle) {
    // TODO populate the message struct in AP_HAL::CAN::transmit, not here

    struct AP_HAL::CAN::message_s msg;
    msg.bus_id = _bus;
    msg.id = _angle_command_id;
    msg.rtr = 0;
    msg.extid = 0;
    msg.length = 4;
    memcpy(msg.data, &angle, 4);
    hal.can->transmit(msg);
}

void CAN_Servo::can_recv(uint32_t time_us, const struct AP_HAL::CAN::message_s& msg)
{
    if (msg.id == _angle_feedback_id && msg.length == 4) {
        memcpy(&_angle, msg.data, 4);
    }
}
