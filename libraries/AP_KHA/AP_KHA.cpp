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

extern const AP_HAL::HAL& hal;

AP_KHA *AP_KHA::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_KHA::var_info[] = {

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
    if (!_params.enabled) {
        return;
    }

}

void AP_KHA::update()
{
    if (!_params.enabled) {
        return;
    }
    
}

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


namespace AP {
AP_KHA &kha()
{
    return *AP_KHA::get_singleton();
}
}
#endif // AP_KHA_ENABLED
