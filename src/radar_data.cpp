#include "radar_data.hpp"
#include <gtsam/geometry/Pose2.h>


using namespace Eigen;
using namespace std;


// convert 
Radat unpackRadarData(const radar_driver::Track track, Pose2 robot_pose){
    Radat r;
    r.x = cos(track.angle) * track.range + robot_pose.x;
    r.y = sin(track.angle) * track.range + robot_pose.y;
    r.x_vel = cos(track.angle) * track.rate;
    r.y_vel = sin(track.angle) * track.rate;
    r.angle = track.angle;
    r.width = track.width;
    r.vel = track.rate;
    r.accel = track.accel;
    r.lat_vel = track.late_rate;
    r.range = track.range;
    r.label1 = Symbol('x', rand() % 1000000 + 1);
    r.label2 = Symbol('r', rand() % 1000000 + 1);
    return r;
}

vector<Radat> repackageData(vector<radar_driver::Track> new_data, Pose2 robot_pose){
    vector<Radat> data;
    for(auto nd : new_data){
        data.push_back(unpackRadarData(nd, robot_pose));
    }
    return data;
}