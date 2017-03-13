#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

using Eigen::MatrixXd;

class FusionEKF
{
  public:
    FusionEKF();
    ~FusionEKF() = default;

    /// @brief Run the whole flow of the Kalman Filter from here.
    void ProcessMeasurement(const MeasurementPackage& measurement_pack);

    /// @brief Returns a reference of the Kalman filter instance
    KalmanFilter& GetKalmanFilter() { return ekf_; }

  private:
    void InitializeMembers();
    void InitializeEkf();

    void InitializeWithFirstMasurement(const MeasurementPackage& measurement_pack);

    void PreparePredictionStep(const MeasurementPackage& measurement_pack);
    void PredictionStep();

    void PrepareUpdateStep();
    void UpdateStep(const MeasurementPackage& measurement_pack);

    /// @brief check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    /// @brief previous timestamp
    long long previous_timestamp_;

    /// @brief H and R matrices for laser
    MatrixXd measurement_transition_H_laser_;
    MatrixXd measurement_covariance_R_laser_;

    /// @brief H and R matrices for radar
    MatrixXd measurement_transition_H_radar_jacobian_;
    MatrixXd measurement_covariance_R_radar_;

    /// @brief Kalman Filter update and prediction math lives in here.
    KalmanFilter ekf_;
};
