#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>

#include "radar_driver/RadarTracks.h"
#include "radar_driver/Track.h"

#include "kalman_filter.hpp"

using namespace std;


void sorter(const radar_driver::Track track){
    if(track.moving){
        float[] track_data = delta(track);
        SLAM.update(track_data);
    }else{
        
    }
}



// this calculates x and y for a radar track and packages the useful information for later use
float* data_extract(const radar_driver::Track track){
    // x, y, width, rate, accel, lateral_vel
    float[6] data;
    data[0] = math.cos(track.angle) * track.range;
    data[1] = math.sin(track.angle) * track.range;
    data[2] = track.width;
    data[3] = track.rate;
    data[4] = track.accel;
    data[5] = track.late_rate;
    return data;
}


// this creates listeners for different radar topics to later listen to all radar topics
ros::Subscriber* radarListener(string topic, ros::NodeHandle* n){
    return n->Subscribe(topic, 1000, )
}


int main(int argc, char * argv){
    ros::init(argc, argv, "rad_slam");
    ros::NodeHandle nh;

    // This creates a vector of radar listeners based on list of params in launch
    vector<string> radar_topics;
    nh.getParam("radar_topics", radar_topics);
    vector<ros::Subscriber*> radar_simps;
    for(int i=0; i<radar_topics.size(); i++){
        radar_simps.push(radarListener(radar_topics[i], nh*));
    }

    ros::spin();
    
}