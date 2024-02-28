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
#pragma once

#include <AP_KHA/AP_KHA_config.h>

#if AP_KHA_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#ifndef AP_KHA_GCS_PARAM_COUNT
#define AP_KHA_GCS_PARAM_COUNT 20
#endif


class AP_KHA
{

public:
    // constructor
    AP_KHA();

    // enable as singleton
    static AP_KHA *get_singleton(void) {
        return _singleton;
    }

    void init();
    void update();
    
    // parse mavlink messages
    void handle_msg(GCS_MAVLINK &link, const mavlink_message_t &msg);
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_KHA *_singleton;

    struct {
        AP_Int8 enabled;
#if AP_KHA_GCS_PARAM_COUNT >= 1
        AP_Float gcs_param[AP_KHA_GCS_PARAM_COUNT];
#endif
    } _params;

};

namespace AP {
    AP_KHA &kha();
};
#endif // AP_KHA_ENABLED
