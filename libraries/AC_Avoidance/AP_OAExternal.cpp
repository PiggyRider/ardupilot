#include "AP_OAExternal.h"

// constructor initialises expanding array to use 20 elements per chunk
AP_OAExternal::AP_OAExternal()
{
}

void AP_OAExternal::handle_msg(const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_rsem);

    mavlink_data32_t m;
    mavlink_msg_data32_decode(&msg, &m);

    // decode m.data[0]~m.data[31] into dest.lat, dest.lng
    // m.data[0]~m.data[31]

    // for test
    // Location temp_loc(Vector3f(5000, 5000, 0.0f), Location::AltFrame::ABOVE_ORIGIN);
    // dest.lat = temp_loc.lat;
    // dest.lng = temp_loc.lng;

    last_update_time_ms = AP_HAL::millis();
}

bool AP_OAExternal::get_oaexternal_destination(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new)
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_update_time_ms > 200) {
        return false;
    }

    origin_new = current_loc;

    WITH_SEMAPHORE(_rsem);
    destination_new.lat = dest.lat;
    destination_new.lng = dest.lng;

    return true;
}
