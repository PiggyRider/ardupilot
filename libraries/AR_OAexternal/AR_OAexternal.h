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

#include "AP_HAL/AP_HAL.h"
#include "AP_AHRS/AP_AHRS.h"
#include "AP_SerialManager/AP_SerialManager.h"
#include "AR_OA_Mavlink/common/mavlink.h"   //include mavlink headers, different from GCSmavlink headers. We send messages from uart port, not from gcs

class AR_OAexternal {

public:
    AR_OAexternal();  //constructor

    /* Do not allow copies */
    AR_OAexternal(const AR_OAexternal &other) = delete;
    AR_OAexternal &operator=(const AR_OAexternal&) = delete;

    // init - perform required initialization
    void init(const AP_SerialManager& serial_manager);

    bool update(void);
    bool get_message(Location& mavmsg, uint32_t& last_update_time_ms);

private:

    AP_HAL::UARTDriver *_port;    //UART port
    Location OAdestination;
    uint32_t last_update_time_ms;
};
