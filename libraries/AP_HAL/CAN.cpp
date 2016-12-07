#include "CAN.h"

void AP_HAL::CANDriver::register_receiver(uint8_t bus_mask, ReceiveCb cb) {
    if (_num_receivers == AP_HAL_CAN_MAX_RECEIVERS) {
        return;
    }

    _receivers[_num_receivers].bus_mask = bus_mask;
    _receivers[_num_receivers].cb = cb;

    _num_receivers++;
}

void AP_HAL::CANDriver::process_message(uint32_t recv_time_us, const struct message_s& message)
{
    for (uint8_t i=0; i<_num_receivers; i++) {
        if (_receivers[i].bus_mask & (1<<message.bus_id)) {
            _receivers[i].cb(recv_time_us, message);
        }
    }
}

