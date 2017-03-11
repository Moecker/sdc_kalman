#include "kalman_filter.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in)
{
    measurement_transition_H_ = H_in;
    measurement_covariance_R_ = R_in;
}

void KalmanFilter::Predict()
{
    // Reference Lecture 5
    // Predicts state x
    /// @todo What about the u part (noise)?
    state_x_ = state_transition_F_ * state_x_;

    // Predicts covariance P
    MatrixXd Ft = state_transition_F_.transpose();
    state_covariance_P_ = state_transition_F_ * state_covariance_P_ * Ft + process_covariance_Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd& z)
{
    // Update State Estimate and uncertainty, prediction or correction step
    VectorXd z_pred = measurement_transition_H_ * state_x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = measurement_transition_H_.transpose();
    MatrixXd S = measurement_transition_H_ * state_covariance_P_ * Ht + measurement_covariance_R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = state_covariance_P_ * Ht;
    MatrixXd K = PHt * Si;

    // Estimate new state and covariance
    state_x_ = state_x_ + (K * y);

    int x_size = static_cast<int>(state_x_.size());
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    state_covariance_P_ = (I - K * measurement_transition_H_) * state_covariance_P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd& z)
{
    Update(z);
}