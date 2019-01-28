#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/Semaphore.h>

#include "AP_OpticalFlow_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <com/hex/equipment/flow/Measurement.hpp>

extern const AP_HAL::HAL& hal;

#define debug_flow_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(MeasurementCb, com::hex::equipment::flow::Measurement);

uint8_t AP_OpticalFlow_UAVCAN::_node_id = 0;
AP_OpticalFlow_UAVCAN* AP_OpticalFlow_UAVCAN::_driver = nullptr;
AP_UAVCAN* AP_OpticalFlow_UAVCAN::_ap_uavcan = nullptr;
/*
  constructor - registers instance at top Flow driver
 */
AP_OpticalFlow_UAVCAN::AP_OpticalFlow_UAVCAN(OpticalFlow &flow) :
    OpticalFlow_backend(flow)
{}

void AP_OpticalFlow_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb> *measurement_listener;
    measurement_listener = new uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb>(*node);
    // Msg Handler
    const int measurement_listener_res = measurement_listener->start(MeasurementCb(ap_uavcan, &handle_measurement));
    if (measurement_listener_res < 0) {
        AP_HAL::panic("UAVCAN Flow subscriber start problem\n\r");
        return;
    }
}

AP_OpticalFlow_UAVCAN* AP_OpticalFlow_UAVCAN::detect(OpticalFlow &flow)
{
    if (_ap_uavcan == nullptr) {
        return nullptr;
    }
    if (_driver == nullptr) {
        _driver = new AP_OpticalFlow_UAVCAN(flow);
        if (_driver == nullptr) {
            hal.console->printf (
                                "Failed register UAVCAN Flow Node %d on Bus %d\n",
                                _node_id,
                                _ap_uavcan->get_driver_index());
        } else {
            hal.console->printf (
                                "Registered UAVCAN Flow Node %d on Bus %d\n",
                                _node_id,
                                _ap_uavcan->get_driver_index());
        }
    }
    return _driver;
}

void AP_OpticalFlow_UAVCAN::handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb)
{
    if (_driver == nullptr) {
        _ap_uavcan = ap_uavcan;
        _node_id = node_id;
        return;
    }
    _driver->flowRate = Vector2f(cb.msg->flow_integral[0], cb.msg->flow_integral[1]);
    _driver->bodyRate = Vector2f(cb.msg->rate_gyro_integral[0], cb.msg->rate_gyro_integral[1]);
    _driver->integral_time = cb.msg->integration_interval;
    _driver->surface_quality = cb.msg->quality;
    _driver->_push_state();
}

void AP_OpticalFlow_UAVCAN::update()
{}

// Read the sensor
void AP_OpticalFlow_UAVCAN::_push_state(void)
{
    struct OpticalFlow::OpticalFlow_state state;
    WITH_SEMAPHORE(_sem_flow);
    const Vector2f flowScaler = _flowScaler();

    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
    float integralToRate = 1.0f / integral_time;

    state.flowRate = Vector2f(flowRate.x * flowScaleFactorX,
                                flowRate.y * flowScaleFactorY) * integralToRate;
    state.bodyRate = bodyRate * integralToRate;
    state.surface_quality = surface_quality;
    _applyYaw(state.flowRate);
    _applyYaw(state.bodyRate);
    // hal.console->printf("DRV: %u %f %f\n", state.surface_quality, flowRate.length(), bodyRate.length());
    _update_frontend(state);
}

#endif // HAL_WITH_UAVCAN

