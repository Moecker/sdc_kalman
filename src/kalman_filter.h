#pragma once

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter
{
  public:
    KalmanFilter();
    virtual ~KalmanFilter();

    /// @brief Init Initializes Kalman filter
    void Init(MatrixXd& H_in, MatrixXd& R_in);

    /// @brief Prediction Predicts the state and the state covariance using the process model
    /// @param delta_T Time between k and k+1 in s
    void Predict();

    /// @brief Updates the state by using standard Kalman Filter equations
    /// @param z The measurement at k+1
    void Update(const VectorXd& z);

    /// @brief Updates the state by using Extended Kalman Filter equations
    /// @param z The measurement at k+1
    void UpdateEKF(const VectorXd& z);

    VectorXd state_x_;
    MatrixXd state_covariance_P_;
    MatrixXd state_transition_F_;
    MatrixXd process_covariance_Q_;
    MatrixXd measurement_transition_H_;
    MatrixXd measurement_covariance_R_;
};
