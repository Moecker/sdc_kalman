#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    /**
    TODO: (progress)
      * Finish initializing the FusionEKF.
    */

    // create a 4D state vector, we don't know yet the values of the x state
    ekf_.x_ = VectorXd(4);

    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

    // measurement covariance
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ << 0.0225, 0, 0, 0.0225;

    // measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1, 0, 0, 0, 0, 1, 0, 0;

    // the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
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
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    UpdateStep(measurement_pack);

    // print the output
    cout << "New Mean x_: \n" << ekf_.x_ << endl;
    cout << "New Covariance P_: \n" << ekf_.P_ << endl;
}

void FusionEKF::UpdateStep(const MeasurementPackage& measurement_pack)
{
    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        // Laser updates
        ekf_.Update(measurement_pack.raw_measurements_);
    }
}

void FusionEKF::PreparePredictionStep(const MeasurementPackage& measurement_pack)
{
    /**
     TODO: (progress)
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
     */

    // compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0F;  // dt - expressed in seconds
    cout << "Elapsed time dt: " << dt << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    // Modify the F matrix so that the time is integrated
    // F is of shape (4, 4)
    //  |0   1   2   3
    // -|--------------
    // 0|0   0   0   0   
    // 1|0   0   0   0 
    // 2|*   0   0   0 
    // 3|0   *   0   0 
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // Helper variables for process covariance matrix
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    float noise_ax = 0.0F;
    float noise_ay = 0.0F;

    // set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0, 0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
        dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0, 0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
}

void FusionEKF::InitializeWithFirstMasurement(const MeasurementPackage& measurement_pack)
{
    /**
    TODO: (progress)
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    cout << "EKF Initialized to: \n" << ekf_.x_ << endl;

    // Kalman is being initialized
    ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        /**
        Initialize state.
        */
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
}
