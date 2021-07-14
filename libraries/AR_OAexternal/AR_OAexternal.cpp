/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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
   Computer OAexternal library
*/

#include "AR_OAexternal.h"
#include <AP_SerialManager/AP_SerialManager.h>

#include <AP_Vehicle/AP_Vehicle.h>

#define AP_SERIALMANAGER_MAVLINK_BAUD           57600
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX     256

//constructor
AR_OAexternal::AR_OAexternal()
{
    _port = NULL;
}

/*
 * init - perform required initialization
 */
void AR_OAexternal::init(const AP_SerialManager& serial_manager)
{
    //check and find out2 which port is our OA assistant computer
    if((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OAcomputer, 0)))
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_MAVLINK_BAUD, AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX, AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX);
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                           "Serial port find");
        return;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                    "Can't find serial port");
    return;
}

bool AR_OAexternal::update(void)
{
    //receive & decode mavlink pack,and translate into vector3f, set desire location
    //plus send current location and velocity to computer for OA algorithm
    if(_port == NULL)
    {
        return false;
    }

    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;

    int16_t numc = _port->available();
    uint8_t temdata=0;
    mavlink_global_position_int_t global_position;

    //mavlink_system_t mavlink_system = {
           //1, // System ID (1-255)
           //1  // Component ID (a MAV_COMPONENT value)
           //};
           mavlink_message_t sndmsg;

           //for test
           int32_t lat = 5000;
           int32_t lon = 5000;
           int32_t alt = 0;
           uint8_t buf[36];
               mavlink_msg_global_position_int_pack(1, 1, &sndmsg, 0, lat, lon, alt, 0, 0, 0, 0, 0);
               uint16_t len=mavlink_msg_to_send_buffer(buf, &sndmsg);

          _port->write(buf, len+12);

    if(!numc == 0)
    {
        for(int16_t i=0;i<numc;i++)
        {
            temdata = _port->read();
        }

        if(mavlink_parse_char(chan, temdata, &msg, &status))
        {
            switch(msg.msgid)
            {
                  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
                  {
                      // Get all fields in payload (into global_position)
                      mavlink_msg_global_position_int_decode(&msg, &global_position);
                  }
                  OAdestination = Location(global_position.lat,global_position.lon,global_position.alt, Location::AltFrame::ABOVE_ORIGIN);
                  last_update_time_ms = AP_HAL::millis();
                      break;
                  default:
                      break;
            }
        }
    }else
    {
        OAdestination.lat = 0;
        return false;
    }



    return true;
}

bool AR_OAexternal::get_message(Location& mavmsg, uint32_t& last_update_time)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_time_ms > 200)
    {
        return false;
    }

    mavmsg = OAdestination;
    last_update_time = last_update_time_ms;
    return true;
}
