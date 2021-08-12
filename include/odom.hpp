// includes 
#include <chrono>
#include <iostream>
#include <boost>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <gtsam/geometry/Pose3.h>

using namespace Eigen;
using namespace std;

class Odom{
    public:
        Odom(float x=0.0, float y=0.0, float z=0.0, float q_x=0.0, float q_y=0.0, float q_z=0.0, float q_w=1.0);
        ~Odom();
        void update();
        // use a quaternion to represent the orientation and because i like pain
        Quaternion orientation();
        // use a vector to represent the position - nothing special 
        vector3f position();

        Pose3 PreviousPose;

        auto t_prev_pose = std::chrono::high_resolution_clock::now();

    private:

        // position and velocities
        float x = 0.0, y = 0.0, z = 0.0;
        float v_x = 0.0, v_y = 0.0, v_z = 0.0;
        // w, x, y, z
        Quaternion o(Quaternion(0, 0, 0, 1));

}