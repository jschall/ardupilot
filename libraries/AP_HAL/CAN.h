#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"
#include "utility/functor.h"

#define AP_HAL_CAN_MAX_RECEIVERS 8

class AP_HAL::CANDriver {
public:
    FUNCTOR_TYPEDEF(ReceiveCb, void, uint32_t, const struct message_s&);
    struct message_s {
        uint8_t bus_id;
        uint32_t id;
        uint8_t rtr : 1;
        uint8_t extid : 1;
        uint8_t length : 4;
        uint8_t data[8];
    };

    void register_receiver(uint8_t bus_mask, ReceiveCb cb);

    virtual bool transmit(const struct message_s& message) = 0;

protected:
    void process_message(uint32_t recv_time_us, const struct message_s& message);

private:
    struct receiver_s {
        uint8_t bus_mask;
        ReceiveCb cb;
    };

    uint8_t _num_receivers;
    struct receiver_s _receivers[AP_HAL_CAN_MAX_RECEIVERS];
};
