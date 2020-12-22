#include "ros/ros.h"
#include "radar_driver/Track.h"

#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
using namespace cv;

// this is a struct for holding radar data and converting radar data
struct Radat{
    float x, y, x_vel, y_vel, width, vel, accel, lat_vel;
};

struct Full_Frame{
    vector<Radat> raw_data;
    Mat radar_img;
    float[2] ego_vels;
}

Radat unpackRadarData(const radar_driver::Track track);