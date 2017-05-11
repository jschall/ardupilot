/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RangeFinder_DevantechI2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_DevantechI2C::AP_RangeFinder_DevantechI2C(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_ranger, instance, _state)
    , _dev(std::move(dev)) {}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_DevantechI2C::detect(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_RangeFinder_DevantechI2C *sensor
        = new AP_RangeFinder_DevantechI2C(_ranger, instance, _state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        if (!sensor->request_reading()) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }

        hal.scheduler->delay(75);

        uint16_t reading_cm;
        if (!sensor->get_reading(reading_cm)) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    return sensor;
}

void AP_RangeFinder_DevantechI2C::init()
{
    // call timer() at ~13Hz
    _dev->register_periodic_callback(75000, FUNCTOR_BIND_MEMBER(&AP_RangeFinder_DevantechI2C::timer, void));
}

bool AP_RangeFinder_DevantechI2C::request_reading(void)
{
    return _dev->transfer((const uint8_t*)"\x00\x51", 2, nullptr, 0);
}


// read - return last value measured by sensor
bool AP_RangeFinder_DevantechI2C::get_reading(uint16_t &reading_cm)
{
    const uint8_t high_byte_reg = 0x02;

    union {
        uint8_t bytes[2];
        be16_t val_be16;
    };

    bool ret = _dev->transfer(&high_byte_reg, 1, bytes, 2);
    if (ret) {
        reading_cm = be16toh(val_be16);
        return true;
    }

    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_DevantechI2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_DevantechI2C::timer(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }

    // Kick off the next reading
    request_reading();
}
