#include <AP_HAL/AP_HAL.h>

class CAN_Servo {
public:
    CAN_Servo(uint8_t bus, uint32_t angle_command_id, uint32_t angle_feedback_id);
    void send_angle_target(float angle);
private:
    void can_recv(uint32_t time_us, const struct AP_HAL::CAN::message_s& msg);
    float _angle;
    uint32_t _angle_command_id;
    uint32_t _angle_feedback_id;
    uint8_t _bus;
};
