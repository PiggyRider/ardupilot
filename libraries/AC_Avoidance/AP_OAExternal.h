#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/AP_ExpandingArray.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

/*
 * Visibility graph used by Dijkstra's algorithm for path planning around fence, stay-out zones and moving obstacles
 */
class AP_OAExternal {
public:
    AP_OAExternal();

    /* Do not allow copies */
    AP_OAExternal(const AP_OAExternal &other) = delete;
    AP_OAExternal &operator=(const AP_OAExternal&) = delete;

    void handle_msg(const mavlink_message_t &msg);

    bool get_oaexternal_destination(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new);

private:
    HAL_Semaphore _rsem;
    uint32_t last_update_time_ms;
    Location dest;
};
