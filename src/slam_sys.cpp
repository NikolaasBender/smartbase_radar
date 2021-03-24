#include "slam_sys.hpp"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// used this paper https://arxiv.org/pdf/2005.02198.pdf
// surf feature extractor 
// http://opencv-tutorials-hub.blogspot.com/2016/03/feature-detection-using-SURF-code-with-example.html 


SLAM::SLAM(){
    
    graph.add(PriorFactor(Transform3(0.0, 0.0, 0.0)));
    initials.insert(Symbol('x', 0), Pose2(0.0, 0.0, 0.0));

}


// this ingests and processes new data for the slam system
void SLAM::Update(vector<radar_driver::Track> new_data){
    // make a full frame to store a bunch of data
    Full_Frame ff;

    // unpack radar data
    ff.raw_data = repackageData(new_data, graph.current_pose); // current pose needs to be changed to be accurate.

    // calculate ego vels of vehicle
    Vector2f ego_motion = EgoMotion(ff.raw_data);
    ff.ego_vels[0] = ego_motion[0];
    ff.ego_vels[1] = ego_motion[1];

    Pose2 odometry = Pose2();
    // this needs to be tuned
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.2; 0.2));
    graph.add(BetweenFactor<Pose2>(s1, s2, odometry, odometryNoise));
    
    for(int i = 0; i < ff.raw_data.size(); i++){
        // rot2 needs to be in radians
        graph.add(BearingRangeFactor2D(ff.raw_data[i].label1, ff.raw_data[i].label2, Rot2(ff.raw_data[i].angle), ff.raw_data[i].range, noiseModel));
    }
    
    // optimize
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();

}


// =====================================
// POSE TRACKING
// =====================================
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