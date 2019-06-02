/** OpenARK UKF usage example  */
#include<iostream>
#include<Eigen/Core>

// OpenARK Libraries
#include "UKF.h"

using namespace ark;

/** Toy UKF model for tracking position and attitude, given GPS position + Gyroscope reading as input */
struct ToyTrackerUKFModel {
    /** define state vector: <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 3, 1> StateVec; // the layout is: (pos x 3, vel x 3, angularvel x 3, attitude x 4)

    /** define measurement vector <scalars, 3-vectors, quaternions> */
    typedef kalman::Vector<0, 2, 0> MeasureVec; // the layout is: (pos x 3, vel x 3, angularvel x 3, rotation x 4) 

    static void init(kalman::UKF<ToyTrackerUKFModel> & ukf) {
        // auto-initialize state to zero and covariances to diagonal matrices
        ukf.defaultInitialize(100, std::sqrt(0.1 * 0.01), std::sqrt(10.));

        // adjust the covariances manually
        ukf.stateRootCov.diagonal() <<
            10000, 10000, 10000, 100, 100, 100, 10, 10, 10, 1, 1, 5;
        ukf.stateRootCov = ukf.stateRootCov.llt().matrixU(); // note that we are using sqrt of actual cov matrix
        ukf.measureNoiseRootCov.diagonal().tail<3>().setConstant(std::sqrt(0.05));
        ukf.processNoiseRootCov.diagonal().template head<3>().setConstant(std::sqrt(0.1 * 0.01 * 0.01));
        ukf.processNoiseRootCov.diagonal().template tail<3>().setConstant(std::sqrt(0.1 * 0.01 * 0.01));
        ukf.processNoiseRootCov = ukf.processNoiseRootCov.llt().matrixU(); // note that we are using sqrt of actual cov matrix
    }

    static StateVec dF(const StateVec & state, const Eigen::MatrixXd & u) {
        StateVec out = state;

        /* differentiate the quaternions automatically.
         * Second argument specifies start of angular velocity params in state vector */
        kalman::util::diffQuaternion(out, 6);

        /* differentiate position automatically.
         * arguments are: (output state vector, start of position param, end of position param, beginning of velocity param)  */
        kalman::util::diffPosition(out, 0, 3, 3);
        return out;
    }

    /* measurement model definition */
    static MeasureVec H(const StateVec & state, const Eigen::MatrixXd & u) {
        MeasureVec out;
        // measure the position (GPS)
        out.template head<3>() = state.template head<3>();
        // measure the angular velocity (Gyroscope)
        out.template segment<3>(3) = state.template segment<3>(6);
        return out;
    }
};

int main(int argc, char ** argv) {
    // construct the SR-UKF
    kalman::UKF<ToyTrackerUKFModel> ukf;

    ToyTrackerUKFModel::MeasureVec m;
    m << 100, -10, -50, 0.5, 0, 0;
    ukf.update(0.01, m, Eigen::MatrixXd()); // apply an update: (time since last update, measurement, input)
    std::cout << ukf.state << "\n" << ukf.stateRootCov << "\n\n"; // read off state and covariance
    // state at this point should be close to 100, -10, -50, the provided measurement

    m << 150, -10, -50, 0.7, 0, 0;
    ukf.update(0.01, m, Eigen::MatrixXd()); // apply an update: (time since last update, measurement, input)
    std::cout << ukf.state << "\n" << ukf.stateRootCov << std::endl; // read off state and covariance
    // state now should be close to 125, -10, -50 
    return 0;
}
