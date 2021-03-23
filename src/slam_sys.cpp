// #include <ros/ros.h>
// #include <ros/console.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>

// #include <iostream>
// #include <fstream>
// #include <string>
// #include <math.h>
// #include <vector>
// #include <algorithm>
// #include <Eigen/Dense>
// #include <chrono>

// #include "radar_driver/RadarTracks.h"
// #include "radar_driver/Track.h"

// #include "opencv2/core/core.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"

// #include "radar_data.hpp"

#include "slam_sys.hpp"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// used this paper https://arxiv.org/pdf/2005.02198.pdf
// surf feature extractor 
// http://opencv-tutorials-hub.blogspot.com/2016/03/feature-detection-using-SURF-code-with-example.html 


SLAM::SLAM(){
    
}


// this ingests and processes new data for the slam system
void SLAM::Update(vector<radar_driver::Track> new_data){
    // make a full frame to store a bunch of data
    Full_Frame ff;

    // unpack radar data
    ff.raw_data = repackageData(new_data);

    // calculate ego vels of vehicle
    Vector2f ego_motion = EgoMotion(ff.raw_data);
    ff.ego_vels[0] = ego_motion[0];
    ff.ego_vels[1] = ego_motion[1];
    
    // make a radar image
    ff.radar_img = RadarPicture(ff.raw_data);

    // do key point extraction on data
    ff.frame_points = GetKeyPoints(ff.radar_img);

    // calculate difference from last keyframe
    

}


// This creates an "image" based on the radar data
// will need modifications to increase resolution but should be tested first
Mat SLAM::RadarPicture(vector<Radat> data){
    Mat rad_img = Mat::zeros(n, n, CV_8U);
    for(int i = 0; i < data.size(); i++){
        // scale the numbers by a factor of 10 for better resolution
        // use 255 as it is max value of unsigned char in c++
        // APPARENTLY ITS (y, x)
        rad_img.at<int>((int)round(data[i].y * scale), (int)round(data[i].x * scale)) = 255;
    }
    return rad_img;
}


// =====================================
// POSE TRACKING
// =====================================
// feature extraction
vector<KeyPoint> SLAM::GetKeyPoints(Mat rad_img){
    int minHessian = 40;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints;
    detector->detect( rad_img, keypoints );
    return keypoints;
}


// track refrence frame
MatrixXd SLAM::GetMovemntProb(vector<KeyPoint> poi){
    auto now = chrono::high_resolution_clock::now();

    // get the difference in time between frames and convert to seconds 
    chrono::duration<float> dt = now - keyframe_time;
    
    // project the keyframe into the future based on motion prior
    Mat kf_pri = Mat::zeros(n, n, CV_8U);
    for(auto d : last_keyframe.frame_points){
        float proj_x = (float)d.pt.x + (pose[2] * dt.count()) ;
        float proj_y = (float)d.pt.y + (pose[3] * dt.count());
        // APPARENTLY ITS (y, x)
        kf_pri.at<int>(proj_y * scale, proj_x * scale) = 255;
    }
}


// calculate egomotion from points
VectorXf SLAM::EgoMotion(vector<Radat> data){
    MatrixXf angle_mtx(data.size());
    VectorXf vel_vec(data.size());
    for(int i = 0; i < data.size(); i++){
        angle_mtx(i, 0) = cos(data.at(i).angle);
        angle_mtx(i, 1) = sin(data.at(i).angle);
        vel_vec(i) = data[i].vel;
    }
    Vector2f x;
    // SVD solution, most accurate, slowest
    x = angle_mtx.bdcSvd(ComputeThinU | ComputeThinV).solve(vel_vec);
    // QR solution, kind fast, kinda accurate
    // x = angle_mtx.colPivHouseholderQr().solve(vel_vec);
    // Normal equations, fast, least accurate
    // x = (angle_mtx.transpose() * angle_mtx).ldlt().solve(angle_mtx.transpose() * vel_vec);
    return x;
}


// track local map
// This is equ 3 in the paper
void SLAM::KeyFrameMatch(MatrixXf frame, vector<KeyPoint> poi){
    // initialize adjacency matrix
    MatrixXb G(last_keyframe.frame_points.size(), poi.size());
    // abs(norm(frame - key_frame)) < sig_c;
    // there might be two ways to do this, one with a single for loop, another 
    for(int i )

}


// new keyframe decision
bool SLAM::NewKeyFrameDecision(){

    return 0;
}


// =====================================
// LOCAL MAPPING
// =====================================
// process new keyframe


// old map point culling


// new map point creation


// local bundle adjust