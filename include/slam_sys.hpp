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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include "radar_driver/RadarTracks.h"
#include "radar_driver/Track.h"

#include "radar_data.hpp"

using namespace Eigen;
using namespace std;
using namespace gtsam;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PtrCloud;

// used this paper https://arxiv.org/pdf/2005.02198.pdf

class SLAM{
    public:
        SLAM();
        ~SLAM();
    private:

        Odom odometry(0, 0, 0, 0, 0, 0);


        vector<radar_driver::Track> newest_data;
        PtrCloud newest_cloud new(PointCloud); 

        vector<radar_driver::Track> not_newest_data;  // may not be needed
        PtrCloud not_newest_cloud new(PointCloud);


        // Create a factor graph container
        NonLinearFactorGraph graph;

        void Update(vector<radar_driver::Track> new_data);

        // this computes the ego velocity of the vehicle
        Vector2f EgoMotion(vector<Radat> data);

        Values initials;

        // odometry measurement noise model (covariance matrix)
        noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.0));

        // tune this for the radar returns
        // noiseModel = noiseModel.Diagonal.Sigmas([0.1; 0.2, 0.0]);
        
        
        GaussNewtonParams parameters;

};


