#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <chrono>
#include <gtsam>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include "radar_driver/RadarTracks.h"
#include "radar_driver/Track.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "radar_data.hpp"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace gtsam;

// used this paper https://arxiv.org/pdf/2005.02198.pdf

class SLAM{
    public:
        SLAM();
        ~SLAM();
    private:
        // this is used for radar image sizing
        // operating area 86m x 60m 
        // (60m, +/-45 deg) 
        int n = 860;
        // also used for image things
        float scale = 10;
        // x_pos, y_pos, x_vel, y_vel, others???
        VectorXf pose;
        // MatrixXf key_frame();
        Full_Frame last_keyframe;
        // used for equ 3 in paper, idk what this should be set to
        float sig_c;
        // needs to be expanded to include size
        Mat RadarPicture(vector<Radat> data);
        // relative transform between keyframe and current frame
        MatrixXd relative_transform;

        void Update(vector<radar_driver::Track> new_data);

        // this computes the ego velocity of the vehicle
        VectorXf EgoMotion(vector<Radat> data);

        // POSE TRACKING
        // feature extraction
        vector<KeyPoint> GetKeyPoints(Mat rad_img);
        // track refrence frame
        MatrixXd GetMovemntProb(vector<KeyPoint> poi);
        // track local map
        void KeyFrameMatch(MatrixXd frame);
        // new keyframe decision
        bool NewKeyFrameDecision(void);

        Values initials;

        // Create a factor graph container
        NonLinearFactorGraph graph;

        // odometry measurement noise model (covariance matrix)
        // noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

        // tune this for the radar returns
        noiseModel = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
        
        
        GaussNewtonParams parameters;

};