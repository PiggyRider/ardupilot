#include "mode.h"
#include "Rover.h"

bool ModeStarWP::_enter()
{
    // set desired location to reasonable stopping point
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
        return false;
    }

    //generate star path when mode is starting
    path_num=0;
    generate_path();

    // initialise waypoint speed
    g2.wp_nav.set_desired_speed_to_default();

    return true;
}

void ModeStarWP::update()
{
    if (!g2.wp_nav.reached_destination()) {
    // update navigation controller
        navigate_to_waypoint();
    } else {
        if(path_num<6){
            path_num ++;
            if(!g2.wp_nav.set_desired_location(path[path_num]))
            {
                return;
            }
            // update distance to destination
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
        } else {
            stop_vehicle();
        }
    }
}

void ModeStarWP::generate_path()
{
    float radius_cm = 1000.0;

    Location TemperaryLoc;
    if(!g2.wp_nav.get_stopping_location(TemperaryLoc))
    {
        return;
    }
    path[0] = TemperaryLoc;

    Vector3f tem_vec;
    if(!path[0].get_vector_from_origin_NEU(tem_vec))
    {
        return;
    }
    path[1] = Location(tem_vec+Vector3f(1.0f, 0, 0) * radius_cm, Location::AltFrame::ABOVE_ORIGIN);
    path[2] = Location(tem_vec+Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm, Location::AltFrame::ABOVE_ORIGIN);
    path[3] = Location(tem_vec+Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm, Location::AltFrame::ABOVE_ORIGIN);
    path[4] = Location(tem_vec+Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm, Location::AltFrame::ABOVE_ORIGIN);
    path[5] = Location(tem_vec+Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm, Location::AltFrame::ABOVE_ORIGIN);
    //path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    //path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    //path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    //path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    //path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}
