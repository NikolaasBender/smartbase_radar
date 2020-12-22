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


void sorter(const radar_driver::RadarTracks radar_tracks){
    vector<radar_driver::Track> stationary;
    vector<radar_driver::Track> moving;
    for(int i = 0; i < radar_tracks.tracks.size(); i++){
        radar_driver::Track r = radar_tracks.tracks[i];
        if(r.moving){
            moving.pushback(r);
        }else{
            stationary.pushback(r);
        }
    }
    SLAM_SYS.update(stationary);
    
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