#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_DevantechI2C : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

private:
    // constructor
    AP_RangeFinder_DevantechI2C(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void init();
    void timer();

    // get a reading
    bool request_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
