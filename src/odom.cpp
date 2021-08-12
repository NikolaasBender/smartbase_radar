// initialize odometry
void Odom::Odom(float x, float y, float z, float r, float p, float yaw){

}

// update odometry from smoothed graph data
void Odom::Update(float x, float y, float z, float r, float p, float yaw){

}


// This update takes a transformation matrix
void Odom::UpdateMeas(Matrix4f mtx, Vector3f vels){
    x += mtx(3,0);
    y += mtx(3,1);
    z += mtx(3,2);
    v_x = vels[0];
    v_y = vels[1];
    // calculate elapsed time
    dt = std::chrono::high_resolution_clock::now() - t_prev_pose;
    // integrate velocity over time to check pose (kind of a check position)
    dx += v_x * dt;
    dy += v_y * dt;
    dz += v_z * dt;
    float err = ((x-dx) + (y-dy) + (z-dz) / 3)^2;
    // if error is too high, do something
    if(err > 0.1){
        std::cout << "odom error: " << err << std::endl;
    }
    Matrix3f rot = mtx.block<3,3>(0,0);
    Quaternion<float> q(Quaternion(rot));
    o += q;
    delete rot;
    delete q;

}

// return position
// x,y,z
Vector3f Odom::position(){
    return Vector3f(x, y, z);
}

// return orientation
// r,p,yaw are in radians
Quaternion Odom::orientation(){
    return o;
}

Pose3 Odom::GetPose(){
    return Pose3(position(), orientation());
}

// only save diffs between keyframes to save on space but make pretty map generation easier