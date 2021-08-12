#include "slam_sys.hpp"

SLAM::SLAM(){
    
    graph.add(PriorFactor(Transform3(0.0, 0.0, 0.0)));
    initials.insert(Symbol('x', 0), Pose2(0.0, 0.0, 0.0));

    // initialize odometry class
    Odom odometry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

}


// this ingests and processes new data for the slam system
void SLAM::Update(vector<radar_driver::Track> new_data){
    // clean data
    for(int i = 0; i < new_data.size(); i++){
        // maybe use odometry to calculate current velocity and the velocities of radar returns
        // maybe like this abs(odometry.vel()-new_data[i].vel()) < 1.0
        if(new_data[i].is_moving()){
            new_data.erase(new_data.begin()+i);
            i--;
        }
    }

    newest_data = new_data;

    //  calculate ego vels of vehicle
    Vector2f ego_motion = EgoMotion();
    odometry.UpdateEgo(ego_motion);

    // calculate tf between frames
    Matrix4f tf_matrix = Tf();
    odometry.UpdateMtx(tf_matrix);

    // make a keyframe (node) decision
    if(keyframe_decision(newest_data)){
        // make a keyframe
        KeyFrame keyframe(newest_data);
        keyframes.push_back(keyframe);
        // using a 3d pose as just 2d

        graph.emplace_shared<BetweenFactor<Pose3>>(keyframes.size()-1, keyframes.size(), Pose3(odometry.position()[0], odometry.position()[1], 0, ), odomModel);

    }


    //  update the graph with the new data

    Pose2 odometry = Pose2();
    // this needs to be tuned
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.2; 0.2));
    odometry = Pose2(odometry.position() + Vector2(ego_motion[0], ego_motion[1]), odometry.orientation());
    graph.add(BetweenFactor<Pose2>(s1, s2, odometry, odometryNoise));
    
    for(int i = 0; i < ff.raw_data.size(); i++){
        // rot2 needs to be in radians
        graph.add(BearingRangeFactor2D(ff.raw_data[i].label1, ff.raw_data[i].label2, Rot2(ff.raw_data[i].angle), ff.raw_data[i].range, noiseModel));
    }
    
    // optimize
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();

}


// calculate egomotion from points
// returns x,y velocity
VectorXf SLAM::EgoMotion(){
    MatrixXf angle_mtx(newest_data.size());
    VectorXf vel_vec(newest_data.size());
    for(int i = 0; i < newest_data.size(); i++){
        angle_mtx(i, 0) = cos(newest_data.at(i).angle);
        angle_mtx(i, 1) = sin(newest_data.at(i).angle);
        vel_vec(i) = newest_data[i].vel;
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



//calculate the transform between two frames using pcl
Matrix4f SLAM::Tf(){

    // newest_cloud -> not_newest_cloud
    // clear newest_cloud
    not_newest_cloud->clear();
    pcl::copyPointCloud(*newest_cloud, *not_newest_cloud);
    newest_cloud->clear();
    // pass global vars of coud and data to converter
    tracks2cloud(newest_data, newest_cloud);


    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource(not_newest_cloud);
    icp.setInputTarget(newest_cloud);
    
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    
    // Perform the alignment
    icp.align (cloud_source_registered);
    
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    
    return transformation;

}


// turn radar tracks into a cloud
void SLAM::tracks2cloud(vector<radar_driver::Track> *data, PtrCloud cloud){
    // interate over data
    for(int i = 0; i < data.size(); i++){
        vector<float> conv_point = toxy(data[i]);
        // add data point to cloud
        pcl::PointXYZ point;
        point.x = conv_point[0];
        point.y = conv_point[1];
        point.x = 0;        // because this is 2d for now
        cloud->points.pushback(point);
    }
}

// convert range and azimuth to xy coordinates
// 0 x
// 1 y
vector<float> SLAM::toxy(radar_driver::Track point){
    vector<float> xy;
    xy.pushbakc(cos(point.angle) * point.range);
    xy.pushback(sin(point.angle) * point.range);
    return xy;
} 

