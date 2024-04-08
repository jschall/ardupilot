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
/*
  simple plane simulator class
*/

#pragma once

#include "SIM_Plane.h"
#include "SIM_Frame.h"

namespace SITL {

/*
  a very simple plane simulator
 */
class K1000 : public Aircraft {
public:
    K1000(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new K1000(frame_str);
    }

protected:
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 1.696;
        float b = 5.0;
        float c = 0.36;
        float c_lift_0 = 0.5383;
        float c_lift_deltae = 0;
        float c_lift_a = 5.8455;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.66;
        float alpha_stall = 0.1745;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.0164;
        float c_y_0 = 0;
        float c_y_b = -0.2987;
        float c_y_p = -0.0541;
        float c_y_r = 0.1917;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.1394;
        float c_l_0 = 0;
        float c_l_p = -0.6622;
        float c_l_b = -0.0651;
        float c_l_r = 0.1536;
        float c_l_deltaa = 0.281;
        float c_l_deltar = -0.0035;
        float c_m_0 = 0.0974;
        float c_m_a = -1.166;
        float c_m_q = -30.21;
        float c_m_deltae = 1.521;
        float c_n_0 = 0;
        float c_n_b = 0.0857;
        float c_n_p = 0.0754;
        float c_n_r = -0.0725;
        float c_n_deltaa = -6.144e-04;
        float c_n_deltar = 0.0447;
        float deltaa_max = 0.2681;
        float deltaa_min = 0.1745;
        float deltae_max = 0.2681;
        float deltar_max = 0.2681;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.215, 0, -0.05};
    } coefficient;
    
    bool is_vtol() const { return frame != nullptr; }

    bool in_launch = false;
    float launch_accel = 1;
    float launch_time = 20;
    uint64_t launch_start_ms;

    Matrix3f inertia_matrix = Matrix3f(1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0);

    Matrix3f vtol_inertia_matrix = Matrix3f(1.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0,
                                            0.0, 0.0, 1.0);

    // Calculated once at startup
    Matrix3f inv_inertia_matrix;

    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &moment, Vector3f &force);

private:
    Frame *frame;
};

} // namespace SITL
