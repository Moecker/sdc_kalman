#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF()
        : ekf_(),
          is_initialized_(false),
          previous_timestamp_(0),
          measurement_transition_H_laser_(MatrixXd(2, 4)),
          measurement_covariance_R_laser_(MatrixXd(2, 2)),
          measurement_transition_H_radar_jacobian_(MatrixXd(3, 4)),
          measurement_covariance_R_radar_(MatrixXd(3, 3))
{
    // Laser:
    const double kLaserUncertainty = 0.0225;
    // clang-format off
    measurement_transition_H_laser_ << 1, 0, 0, 0,
                                       0, 1, 0, 0;
    measurement_covariance_R_laser_ << kLaserUncertainty, 0,
                                       0, kLaserUncertainty;
    // clang-format on

    // Radar:
    const double kRadarUncertainty = 0.1;
    // clang-format off
    measurement_covariance_R_radar_ << kRadarUncertainty, 0, 0,
                                       0, kRadarUncertainty, 0,
                                       0, 0, kRadarUncertainty;
    // clang-format on

    // Create a 4D state vector, we don't know yet the values of the x state
    ekf_.state_x_ = VectorXd(4);

    // Make the position very certain and velocity very uncertain for the beginning
    ekf_.state_covariance_P_ = MatrixXd(4, 4);
    const double kPosUncertainty = 1.0;
    const double kVelUncertainty = 1000.0;
    // clang-format off
    ekf_.state_covariance_P_ << kPosUncertainty, 0, 0, 0,
                                0, kPosUncertainty, 0, 0,
                                0, 0, kVelUncertainty, 0,
                                0, 0, 0, kVelUncertainty;
    // clang-format on

    // Those will be filled with either laser of radar covariance / transition
    ekf_.measurement_covariance_R_ = MatrixXd(2, 2);
    ekf_.measurement_transition_H_ = MatrixXd(2, 4);

    // The initial transition matrix F_
    ekf_.state_transition_F_ = MatrixXd(4, 4);
    // clang-format off
    ekf_.state_transition_F_ << 1, 0, 1, 0, 
                                0, 1, 0, 1, 
                                0, 0, 1, 0, 
                                0, 0, 0, 1;
    // clang-format on
}

FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_)
    {
        InitializeWithFirstMasurement(measurement_pack);
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    PreparePredictionStep(measurement_pack);
    PredictionStep();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    PrepareUpdateStep();
    UpdateStep(measurement_pack);

    // print the output
    cout << "New Mean x_: \n" << ekf_.state_x_ << endl;
    cout << "New Covariance P_: \n" << ekf_.state_covariance_P_ << endl;
}

void FusionEKF::PredictionStep()
{
    // ekf_.state_transition_F_ = tools.CalculateJacobianStateTrasition(ekf_.state_x_);
    ekf_.Predict();
}

void FusionEKF::UpdateStep(const MeasurementPackage& measurement_pack)
{
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        cout << "Update with Radar..." << endl;

        measurement_transition_H_radar_jacobian_ = tools.CalculateJacobian(ekf_.state_x_);
        ekf_.Init(measurement_transition_H_radar_jacobian_, measurement_covariance_R_radar_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        // Laser updates
        cout << "Update with Laser..." << endl;

        ekf_.Init(measurement_transition_H_laser_, measurement_covariance_R_laser_);
        ekf_.Update(measurement_pack.raw_measurements_);
    }
}

void FusionEKF::PrepareUpdateStep()
{
    // Nothing to do
}

void FusionEKF::PreparePredictionStep(const MeasurementPackage& measurement_pack)
{
    cout << "Prediction..." << endl;

    // Compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0F;  // dt - expressed in seconds
    cout << "Elapsed time dt: " << dt << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    // Modify the F matrix so that the time is integrated
    // F is of shape (4, 4)
    //  |0   1   2   3
    // -|--------------
    // 0|1   0   dt  0
    // 1|0   1   0   dt
    // 2|0   0   1   0
    // 3|0   0   0   1
    ekf_.state_transition_F_(0, 2) = dt;
    ekf_.state_transition_F_(1, 3) = dt;

    // Helper variables for process covariance matrix
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    float noise_ax = 3.0F;
    float noise_ay = 3.0F;

    // Set the process covariance matrix Q
    ekf_.process_covariance_Q_ = MatrixXd(4, 4);
    // clang-format off
    ekf_.process_covariance_Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
                                  0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
                                  dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
                                  0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
    // clang-format on
}

void FusionEKF::InitializeWithFirstMasurement(const MeasurementPackage& measurement_pack)
{
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Converts radar measurement from polar into euclidean coordinate system
        double ro = measurement_pack.raw_measurements_(0);
        double phi = measurement_pack.raw_measurements_(1);
        auto x = ro * cos(phi);
        auto y = ro * sin(phi);
        ekf_.state_x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        ekf_.state_x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "EKF Initialized to: \n" << ekf_.state_x_ << endl;

    // Done initializing, no need to predict or update
    is_initialized_ = true;
}
