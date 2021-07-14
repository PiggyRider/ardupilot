#include <AC_Avoidance/AP_OAExternal.h>

#define AP_SERIALMANAGER_MAVLINK_BAUD           57600
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_RX     128
#define AP_SERIALMANAGER_MAVLINK_BUFSIZE_TX     256

// constructor initialises expanding array to use 20 elements per chunk
AP_OAExternal::AP_OAExternal()
{
}

void AP_OAExternal::update(Location &oalocation, uint32_t &objecttime)
{
    OAdestination = oalocation;
    last_update_time_ms = objecttime;
}

bool AP_OAExternal::get_oaexternal_destination(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new)
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_update_time_ms > 200) {
        return false;
    }

    origin_new = current_loc;

    WITH_SEMAPHORE(_rsem);
    destination_new.lat = OAdestination.lat;
    destination_new.lng = OAdestination.lng;

    return true;
}
