#include "ros/ros.h"
#include "radar_driver/Track.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;

// this is a struct for holding radar data and converting radar data
struct Radat{
    float x, y, x_vel, y_vel, width, vel, accel, lat_vel, angle;
};

struct Full_Frame{
    vector<Radat> raw_data;
    Mat radar_img;
    float* ego_vels;
    vector<KeyPoint> frame_points;
    bool is_keyframe;
    chrono::time_point<chrono::high_resolution_clock> frame_time;
};

Radat unpackRadarData(const radar_driver::Track track);

vector<Radat> repackageData(vector<radar_driver::Track> new_data);