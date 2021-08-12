// GTSAM headers
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h> // added from demo
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h> // IMPORTANT


using namespace std;
using namespace gtsam;
using namespace gtsamexamples;


int main(int argc, char* argv){
    NonlinearFactorGraph graph;

    // add a gaussian prior to pose x_1
    Pose2 priorMean(0.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // add two odometry factors
    Pose2 odometry(2.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

    Pose2 odometry2(4.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr odometryNoise2 = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.4, 0.1));
    graph.add(BetweenFactor<Pose2>(3, 4, odometry2, odometryNoise2));


    // create bad initial estimates
    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));

    // OPTIMIZE
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    // get cov matricies
    Marginals marginals(graph, result);

    /*
    marginals.marginalCovariance(1)
    
    gets 

    x1 covariance:
        0.09        1.1e-47     5.7e-33
        1.1e-47        0.09     1.9e-17
        5.7e-33     1.9e-17        0.01

    An important fact to note when interpreting these numbers is that 
    covariance matrices are given in relative coordinates, not absolute 
    coordinates. This is because internally GTSAM optimizes for a change 
    with respect to a linearization point, as do all nonlinear optimization 
    libraries.
    */

}