#pragma once

#include "RangeFinder_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_Common/Semaphore.h>

class MeasurementCb;

class AP_RangeFinder_UAVCAN : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_UAVCAN(RangeFinder::RangeFinder_State &state, AP_RangeFinder_Params &params);

    void update() override {}

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_RangeFinder_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new);
    static AP_RangeFinder_Backend* detect(RangeFinder::RangeFinder_State &state, AP_RangeFinder_Params &params);

    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }
private:
    static bool take_registry();
    static void give_registry();

    uint8_t _instance;

    HAL_Semaphore _sem_range;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_RangeFinder_UAVCAN* driver;
    } _detected_modules[RANGEFINDER_MAX_INSTANCES];

    static HAL_Semaphore _sem_registry;
};
