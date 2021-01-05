#include "radar_data.hpp"

using namespace Eigen;
using namespace std;


// convert 
Radat unpackRadarData(const radar_driver::Track track){
    Radat r;
    r.x = cos(track.angle) * track.range;
    r.y = sin(track.angle) * track.range;
    r.x_vel = cos(track.angle) * track.rate;
    r.y_vel = sin(track.angle) * track.rate;
    r.angle = track.angle;
    r.width = track.width;
    r.vel = track.rate;
    r.accel = track.accel;
    r.lat_vel = track.late_rate;
    return r;
}
