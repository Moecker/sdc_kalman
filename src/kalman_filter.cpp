#include "kalman_filter.h"

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
    TODO:
      * predict the state
    */
}

void KalmanFilter::Update(const Eigen::VectorXd& z)
{
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd& z)
{
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
}
