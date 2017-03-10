#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools()
{
}

Tools::~Tools()
{
}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                     const std::vector<Eigen::VectorXd>& ground_truth)
{
    /**
    TODO:
      * Calculate the RMSE here.
    */
    return Eigen::VectorXd(1);
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state)
{
    /**
    TODO: (done)
      * Calculate a Jacobian here.
    */
    MatrixXd Hj(3, 4);
    // recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // pre-compute a set of terms to avoid repeated calculation
    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = (c1 * c2);

    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0, -(py / c1), (px / c1), 0, 0, py * (vx * py - vy * px) / c3,
        px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
