#include "AP_Hydra.h"

void AP_Hydra_SITL::update()
{
    // write torque out
    switch(_instance) {
        case 0:
            _sitl.hydra0_torque = _torque_out;
            break;
        case 1:
            _sitl.hydra1_torque = _torque_out;
            break;
    }

    //::printf("%u %d\n", _instance, _torque_out);

    // read angular position
    uint16_t rotor_pos = 0;
    switch(_instance) {
        case 0:
            rotor_pos = _sitl.state.hydra0_ang_pos;
            break;
        case 1:
            rotor_pos = _sitl.state.hydra1_ang_pos;
            break;
    }

    _rotor_pos_rad = (float)rotor_pos * (2.0f*M_PI)/65536.0f;
    _rotor_pos_update_us = (uint32_t)_sitl.state.timestamp_us;
}
