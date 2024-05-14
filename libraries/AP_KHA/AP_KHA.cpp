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

#include "AP_KHA.h"

#if AP_KHA_ENABLED
#include <AP_BattMonitor/AP_BattMonitor.h>
#include "BatteryChemistryModel.h"

static float soc_ocv_x[] = {0.0, 0.005063014925373088, 0.01613838805970147, 0.02905964179104481, 0.04382680597014932, 0.060439850746268675, 0.07705289552238803, 0.09920364179104468, 0.1268920298507462, 0.15642635820895523, 0.19334423880597018, 0.2357997910447761, 0.2708717910447762, 0.2967142985074628, 0.3244027164179104, 0.34839934328358213, 0.3779336417910447, 0.4037761791044776, 0.4388481492537314, 0.462844776119403, 0.4868414029850746, 0.5182216119402985, 0.5551394925373134, 0.5920573731343284, 0.6289752537313433, 0.6695849253731343, 0.7194240895522388, 0.7581878507462687, 0.7932598507462687, 0.8283318507462687, 0.8615579402985074, 0.9058594029850746, 0.9446231641791045, 0.9815410447761194, 1.0};

static float soc_ocv_y[] = {2.5180000000000002, 2.6487000000000003, 2.75, 2.8668, 2.9681, 3.0693, 3.1550000000000002, 3.2406, 3.3107, 3.373, 3.4198, 3.4587, 3.4899, 3.5132, 3.5288, 3.5444, 3.5678, 3.5911, 3.6145, 3.6456, 3.6612, 3.7001, 3.7313, 3.7702, 3.8014, 3.8403, 3.8793, 3.9104, 3.9494000000000002, 3.9961, 4.027299999999999, 4.0584, 4.074, 4.1051, 4.158};

static BatteryChemistryModelLinearInterpolated chemistry_model(soc_ocv_x, soc_ocv_y, sizeof(soc_ocv_x)/sizeof(soc_ocv_x[0]));

#if KHA_PERIPH_DISTRO
#include <hal.h>
#endif

extern const AP_HAL::HAL& hal;

AP_KHA *AP_KHA::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_KHA::var_info[] = {

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: ENABLE
    // @DisplayName: KHA Enable Features
    // @Description: KHA Enable Features
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_KHA, _params.enabled, 1, AP_PARAM_FLAG_ENABLE),

#if AP_KHA_GCS_PARAM_COUNT >= 1
    // @Param: GCS_PARAM1
    // @DisplayName: GCS_PARAM1
    // @Description: GCS_PARAM1
    AP_GROUPINFO("GCS_PARAM1", 2, AP_KHA, _params.gcs_param[0], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 2
    // @Param: GCS_PARAM2
    // @DisplayName: GCS_PARAM2
    // @Description: GCS_PARAM2
    AP_GROUPINFO("GCS_PARAM2", 3, AP_KHA, _params.gcs_param[1], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 3
    // @Param: GCS_PARAM3
    // @DisplayName: GCS_PARAM3
    // @Description: GCS_PARAM3
    AP_GROUPINFO("GCS_PARAM3", 4, AP_KHA, _params.gcs_param[2], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 4
    // @Param: GCS_PARAM4
    // @DisplayName: GCS_PARAM4
    // @Description: GCS_PARAM4
    AP_GROUPINFO("GCS_PARAM4", 5, AP_KHA, _params.gcs_param[3], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 5
    // @Param: GCS_PARAM5
    // @DisplayName: GCS_PARAM5
    // @Description: GCS_PARAM5
    AP_GROUPINFO("GCS_PARAM5", 6, AP_KHA, _params.gcs_param[4], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 6
    // @Param: GCS_PARAM6
    // @DisplayName: GCS_PARAM6
    // @Description: GCS_PARAM6
    AP_GROUPINFO("GCS_PARAM6", 7, AP_KHA, _params.gcs_param[5], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 7
    // @Param: GCS_PARAM7
    // @DisplayName: GCS_PARAM7
    // @Description: GCS_PARAM7
    AP_GROUPINFO("GCS_PARAM7", 8, AP_KHA, _params.gcs_param[6], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 8
    // @Param: GCS_PARAM8
    // @DisplayName: GCS_PARAM8
    // @Description: GCS_PARAM8
    AP_GROUPINFO("GCS_PARAM8", 9, AP_KHA, _params.gcs_param[7], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 9
    // @Param: GCS_PARAM9
    // @DisplayName: GCS_PARAM9
    // @Description: GCS_PARAM9
    AP_GROUPINFO("GCS_PARAM9", 10, AP_KHA, _params.gcs_param[8], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 10
    // @Param: GCS_PARAM10
    // @DisplayName: GCS_PARAM10
    // @Description: GCS_PARAM10
    AP_GROUPINFO("GCS_PARAM10", 11, AP_KHA, _params.gcs_param[9], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 11
    // @Param: GCS_PARAM11
    // @DisplayName: GCS_PARAM11
    // @Description: GCS_PARAM11
    AP_GROUPINFO("GCS_PARAM11", 12, AP_KHA, _params.gcs_param[10], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 12
    // @Param: GCS_PARAM12
    // @DisplayName: GCS_PARAM12
    // @Description: GCS_PARAM12
    AP_GROUPINFO("GCS_PARAM12", 13, AP_KHA, _params.gcs_param[11], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 13
    // @Param: GCS_PARAM13
    // @DisplayName: GCS_PARAM13
    // @Description: GCS_PARAM13
    AP_GROUPINFO("GCS_PARAM13", 14, AP_KHA, _params.gcs_param[12], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 14
    // @Param: GCS_PARAM14
    // @DisplayName: GCS_PARAM14
    // @Description: GCS_PARAM14
    AP_GROUPINFO("GCS_PARAM14", 15, AP_KHA, _params.gcs_param[13], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 15
    // @Param: GCS_PARAM15
    // @DisplayName: GCS_PARAM15
    // @Description: GCS_PARAM15
    AP_GROUPINFO("GCS_PARAM15", 16, AP_KHA, _params.gcs_param[14], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 16
    // @Param: GCS_PARAM16
    // @DisplayName: GCS_PARAM16
    // @Description: GCS_PARAM16
    AP_GROUPINFO("GCS_PARAM16", 17, AP_KHA, _params.gcs_param[15], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 17
    // @Param: GCS_PARAM17
    // @DisplayName: GCS_PARAM17
    // @Description: GCS_PARAM17
    AP_GROUPINFO("GCS_PARAM17", 18, AP_KHA, _params.gcs_param[16], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 18
    // @Param: GCS_PARAM18
    // @DisplayName: GCS_PARAM18
    // @Description: GCS_PARAM18
    AP_GROUPINFO("GCS_PARAM18", 19, AP_KHA, _params.gcs_param[17], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 19
    // @Param: GCS_PARAM19
    // @DisplayName: GCS_PARAM19
    // @Description: GCS_PARAM19
    AP_GROUPINFO("GCS_PARAM19", 20, AP_KHA, _params.gcs_param[18], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 20
    // @Param: GCS_PARAM20
    // @DisplayName: GCS_PARAM20
    // @Description: GCS_PARAM20
    AP_GROUPINFO("GCS_PARAM20", 21, AP_KHA, _params.gcs_param[19], 0),
#endif
#if AP_KHA_GCS_PARAM_COUNT >= 21
#error "AP_KHA_GCS_PARAM_COUNT == 21 is too large"
#endif

    // @Param: ENDR_PWR
    // @DisplayName: ENDR_PWR
    // @Description: ENDR_PWR
    AP_GROUPINFO("ENDR_PWR", 22, AP_KHA, _params.endure_pwr, 100),
    
    // @Param: ENDR_AUX_PWR
    // @DisplayName: ENDR_AUX_PWR
    // @Description: ENDR_AUX_PWR
    AP_GROUPINFO("ENDR_AUX_PWR", 23, AP_KHA, _params.endure_aux_pwr, 20),

    // @Param: ENDR_ARSP
    // @DisplayName: ENDR_ARSP
    // @Description: ENDR_ARSP
    AP_GROUPINFO("ENDR_ARSP", 24, AP_KHA, _params.endure_arsp, 16),
    
    // @Param: ENDR_MASS
    // @DisplayName: ENDR_MASS
    // @Description: ENDR_MASS
    AP_GROUPINFO("ENDR_MASS", 25, AP_KHA, _params.endure_mass, 18),
    
    // @Param: BATT_CELLS
    // @DisplayName: BATT_CELLS
    // @Description: BATT_CELLS
    AP_GROUPINFO("BATT_CELLS", 26, AP_KHA, _params.battery_cell_count, 6),

    // @Param: THR_LOSS_FIX
    // @DisplayName: THR_LOSS_FIX
    // @Description: THR_LOSS_FIX
    AP_GROUPINFO("THR_LOSS_FIX", 27, AP_KHA, _params.script_throttle_loss_fix, 0),

#elif KHA_PERIPH_DISTRO
    // @Param: DISTRO_CAL
    // @DisplayName: DISTRO_CAL
    // @Description: DISTRO_CAL
    AP_GROUPINFO("DISTRO_CAL", 2, AP_KHA, distro.batt_calibrtaion, 0),

#endif

    AP_GROUPEND
};

AP_KHA::AP_KHA()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_KHA must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_KHA::init()
{
#ifdef HAL_BUILD_AP_PERIPH
    _params.enabled.set(1);
#endif

    distro.batt_calibrtaion.set_and_save(0);

    if (!_params.enabled) {
        return;
    }
}

void AP_KHA::update()
{
    if (!_params.enabled) {
        return;
    }
    
    static uint32_t last_send_ms;
    uint32_t tnow_ms = AP_HAL::millis();
    if (_params.battery_cell_count > 0 && tnow_ms-last_send_ms > 1000) {
        float energy_J;
        if (!AP::battery().energy_remaining_J(energy_J)) {
            // Compute energy remaining from voltage
            // This eliminates error due to coulomb counting drift, but does not account for
            // the state of health of the battery or an incorrectly configured pack capacity
            const float batt_temp = 25; // not currently modeled
            const float SOC = chemistry_model.SOC_from_OCV(AP::battery().voltage_resting_estimate()/_params.battery_cell_count,batt_temp);
            const float pack_capacity_coulombs = AP::battery().pack_capacity_mah() * 3.6; // convert mah to coulombs
            energy_J = pack_capacity_coulombs * chemistry_model.OCV_from_SOC_integral(SOC,batt_temp) * _params.battery_cell_count;
        }
#if HAL_GCS_ENABLED
        gcs().send_named_float("BatERem", energy_J);
#endif
        last_send_ms = tnow_ms;
    }

#if KHA_PERIPH_DISTRO
    distro_calibrate_battery_currents();
#endif
}

#if HAL_GCS_ENABLED
void AP_KHA::handle_msg(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    if (!_params.enabled) {
        return;
    }

    MAV_RESULT result = MAV_RESULT_UNSUPPORTED;

    switch (msg.msgid) {
    case 131300:
        result = MAV_RESULT_ACCEPTED;
        break;

    case 131301 ... 131399:
        result = MAV_RESULT_TEMPORARILY_REJECTED;
        break;
    }

    // send ACK or NAK
    mavlink_msg_command_ack_send(link.get_chan(), msg.msgid, result,
                                 0, 0,
                                 msg.sysid,
                                 msg.compid);
}

MAV_RESULT AP_KHA::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (!_params.enabled) {
        return MAV_RESULT_DENIED;
    }

    MAV_RESULT result = MAV_RESULT_UNSUPPORTED;

    switch (packet.command) {
    case 13130:
        result = MAV_RESULT_ACCEPTED;
        break;

    case 13131 ... 13139:
        result = MAV_RESULT_TEMPORARILY_REJECTED;
        break;
    }
    
    return result;
}
#endif // HAL_GCS_ENABLED

void AP_KHA::distro_calibrate_battery_currents()
{
#if KHA_PERIPH_DISTRO
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t update_interval_ms = 100;
    static uint32_t last_update_ms = 0;
    if (now_ms - last_update_ms < update_interval_ms) {
        // run at 10 Hz
        return;
    }
    last_update_ms = now_ms;
    update_interval_ms = 100;

    const int32_t cal_param = distro.batt_calibrtaion.get();
    static int32_t cal_param_prev = 0;
    static uint32_t zero_cross_count = 0;
    const uint16_t battery_index = (cal_param / 10) - 1;
    const uint16_t expected_amps = cal_param % 10;

    if (cal_param_prev != cal_param) {
        // param changed, init
#ifdef HAL_GPIO_PIN_LED_BATT_CALIBRATE
        palWriteLine(HAL_GPIO_PIN_LED_BATT_CALIBRATE, !HAL_LED_ON);
#endif
        zero_cross_count = 0;

        if (cal_param_prev >= 10 && cal_param_prev <= 69) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Cal STOP");
        }

        if (cal_param >= 10 && cal_param <= 69) {
            // turn on the payload
#ifdef KHA_DISTRO_PAYLOAD_x_ENABLE_PIN_FIRST
            hal.gpio->write(KHA_DISTRO_PAYLOAD_x_ENABLE_PIN_FIRST + battery_index, 1);
#endif
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current cal start on Batt%d, %uA", (int)battery_index + 1, (unsigned)expected_amps);
            // 2s startup delay to ensure stable power
            update_interval_ms = 2000;
        }
        cal_param_prev = cal_param;
        return;
    }

    if (cal_param < 10 || cal_param >= 70) {
        // out-of-range
        return;
    }


    const AP_BattMonitor &batt = AP::battery();
    float current_measured = -1;

    if (!batt.healthy(battery_index) ||
        batt.get_type(battery_index) != AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT ||
        !batt.current_amps(current_measured, battery_index))
    {
        // library is not ready
        // TODO: blink LED with error?
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "batt lib: %u, %u, %.3f", (unsigned)batt.healthy(), (unsigned)batt.get_type(), (double)current_measured);
        return;
    }

    const char* param_str_offset[] = {  "BATT_AMP_OFFSET" ,
                                        "BATT2_AMP_OFFSET",
                                        "BATT3_AMP_OFFSET",
                                        "BATT4_AMP_OFFSET",
                                        "BATT5_AMP_OFFSET",
                                        "BATT6_AMP_OFFSET",
                                        "BATT8_AMP_OFFSET",
                                        "BATT9_AMP_OFFSET",
                                        "BATT9_AMP_OFFSET" };
                                        
    const char* param_str_scaler[] = {  "BATT_AMP_PERVLT" ,
                                        "BATT2_AMP_PERVLT",
                                        "BATT3_AMP_PERVLT",
                                        "BATT4_AMP_PERVLT",
                                        "BATT5_AMP_PERVLT",
                                        "BATT6_AMP_PERVLT",
                                        "BATT7_AMP_PERVLT",
                                        "BATT8_AMP_PERVLT",
                                        "BATT9_AMP_PERVLT" };


    const char* param_str = (expected_amps == 0) ? param_str_offset[battery_index] : param_str_scaler[battery_index];

    float param_value = 0;
    if (!AP_Param::get(param_str, param_value)) {
        // TODO: blink LED with error?
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "failed to get %s", param_str);
        return;
    }

    // TODO: adjust param until current matches expected_amps and track if if we have to change directions on the param

    static float current_lpf = 0;
    const float LPF_coef = 0.95f; // higher means slower. Note, this runs at 10Hz
    current_lpf = current_measured*(LPF_coef) + current_lpf*(1.0f-LPF_coef);

    const float current_error = (current_lpf - expected_amps);
    const bool current_error_positive = (current_error >= 0) ^ (expected_amps == 0);

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%.4f, %.4f, %.4f", (double)current_lpf, (double)current_error, (double)param_value);

    const float change_pct = 0.0001f;
    param_value *= (current_error_positive ? (1.0f - change_pct) : (1.0f + change_pct));

    const float param_value_min = (expected_amps == 0) ? 1 : 10;
    const float param_value_max = (expected_amps == 0) ? 2 : 13;
    param_value = constrain_float(param_value, param_value_min, param_value_max);

    AP_Param::set_by_name(param_str, param_value);

    if (zero_cross_count < 10) {
        // LED Toggle @10Hz
#ifdef HAL_GPIO_PIN_LED_BATT_CALIBRATE
        palToggleLine(HAL_GPIO_PIN_LED_BATT_CALIBRATE);
#endif
        static bool current_error_positive_prev = false;
        if (current_error_positive_prev != current_error_positive) {
            current_error_positive_prev = current_error_positive;
            zero_cross_count++;
        }

    } else {
        // Done: LED SOLID ON
#ifdef HAL_GPIO_PIN_LED_BATT_CALIBRATE
        palWriteLine(HAL_GPIO_PIN_LED_BATT_CALIBRATE, HAL_LED_ON);
#endif
        distro.batt_calibrtaion.set(0);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s = %0.5f, %0.3fA", param_str, (double)param_value, (double)current_lpf);
    }
#endif
}


namespace AP {
AP_KHA &kha()
{
    return *AP_KHA::get_singleton();
}
}
#endif // AP_KHA_ENABLED
