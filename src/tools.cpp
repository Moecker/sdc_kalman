#include "tools.h"
#include <iostream>

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
    return Eigen::VectorXd();
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state)
{
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    return Eigen::MatrixXd();
}
