#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
  public:
    KalmanFilter();
    virtual ~KalmanFilter();

    /// @brief Init Initializes Kalman filter
    void Init(Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in);

    /// @brief Prediction Predicts the state and the state covariance using the process model
    /// @param delta_T Time between k and k+1 in s
    void Predict();

    /// @brief Updates the state by using standard Kalman Filter equations
    /// @param z The measurement at k+1
    void Update(const Eigen::VectorXd& z);

    /// @brief Updates the state by using Extended Kalman Filter equations
    /// @param z The measurement at k+1
    void UpdateEKF(const Eigen::VectorXd& z);

    Eigen::VectorXd state_x_;
    Eigen::MatrixXd state_covariance_P_;
    Eigen::MatrixXd state_transition_F_;
    Eigen::MatrixXd process_covariance_Q_;
    Eigen::MatrixXd measurement_transition_H_;
    Eigen::MatrixXd measurement_covariance_R_;
};

#endif
