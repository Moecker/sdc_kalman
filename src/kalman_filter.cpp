#include "kalman_filter.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(Eigen::VectorXd& x_in,
                        Eigen::MatrixXd& P_in,
                        Eigen::MatrixXd& F_in,
                        Eigen::MatrixXd& H_in,
                        Eigen::MatrixXd& R_in,
                        Eigen::MatrixXd& Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    /**
    TODO: (done)
      * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd& z)
{
    /**
    TODO: (done)
      * update the state by using Kalman Filter equations
    */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    auto x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd& z)
{
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
}
