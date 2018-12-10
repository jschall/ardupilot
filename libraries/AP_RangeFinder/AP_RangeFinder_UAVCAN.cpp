#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_RangeFinder_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/range_sensor/Measurement.hpp>

extern const AP_HAL::HAL& hal;

#define debug_range_finder_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(MeasurementCb, uavcan::equipment::range_sensor::Measurement);

AP_RangeFinder_UAVCAN::DetectedModules AP_RangeFinder_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_RangeFinder_UAVCAN::_sem_registry;

/*
  constructor - registers instance at top RangeFinder driver
 */
AP_RangeFinder_UAVCAN::AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &state, AP_RangeFinder_Params &params) :
    AP_RangeFinder_Backend(state, params)
{}

void AP_RangeFinder_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::range_sensor::Measurement, MeasurementCb> *measurement_listener;
    measurement_listener = new uavcan::Subscriber<uavcan::equipment::range_sensor::Measurement, MeasurementCb>(*node);
    // Msg Handler
    const int measurement_listener_res = measurement_listener->start(MeasurementCb(ap_uavcan, &handle_measurement));
    if (measurement_listener_res < 0) {
        AP_HAL::panic("UAVCAN RangeFinder subscriber start problem\n\r");
        return;
    }
}

bool AP_RangeFinder_UAVCAN::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_RangeFinder_UAVCAN::give_registry()
{
    _sem_registry.give();
}

AP_RangeFinder_Backend* AP_RangeFinder_UAVCAN::detect(RangeFinder::RangeFinder_State &state, AP_RangeFinder_Params &params)
{
    if (!take_registry()) {
        return nullptr;
    }
    AP_RangeFinder_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_RangeFinder_UAVCAN(state, params);
            if (backend == nullptr) {
                debug_range_finder_uavcan(0,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Failed register UAVCAN RangeFinder Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;
                backend->set_status(RangeFinder::RangeFinder_NoData);
                debug_range_finder_uavcan(0,
                                  _detected_modules[i].ap_uavcan->get_driver_index(),
                                  "Registered UAVCAN RangeFinder Node %d on Bus %d\n",
                                  _detected_modules[i].node_id,
                                  _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }
    give_registry();
    return backend;
}

AP_RangeFinder_UAVCAN* AP_RangeFinder_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}

void AP_RangeFinder_UAVCAN::handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb)
{
    if (take_registry()) {
        AP_RangeFinder_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, true);
        if (driver == nullptr) {
            give_registry();
            return;
        }
        {
            WITH_SEMAPHORE(driver->_sem_range);
            switch (cb.msg->reading_type) {
                case uavcan::equipment::range_sensor::Measurement::READING_TYPE_VALID_RANGE:
                {
                    driver->state.distance_cm = cb.msg->range*100.0f;
                    driver->state.last_reading_ms = AP_HAL::millis();
                    driver->update_status();
                    break;
                }
                case uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_CLOSE:
                {
                    driver->set_status(RangeFinder::RangeFinder_OutOfRangeLow);
                    break;
                }
                case uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_FAR:
                {
                    driver->set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
                    break;
                }
                case uavcan::equipment::range_sensor::Measurement::READING_TYPE_UNDEFINED:
                {
                    driver->set_status(RangeFinder::RangeFinder_NoData);
                    break;
                }
                default:
                {
                    driver->set_status(RangeFinder::RangeFinder_NoData);
                    break;
                }
            }
        }
        give_registry();
    }
}

#endif // HAL_WITH_UAVCAN

