#include "ros/ros.h"
#include "radar_driver/Track.h"

// this is a struct for holding radar data and converting radar data
struct Radat{
    float x, y, x_vel, y_vel, width, vel, accel, lat_vel;
};

Radat unpackRadarData(const radar_driver::Track track);