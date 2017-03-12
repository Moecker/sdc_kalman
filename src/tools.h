#pragma once

#include <vector>
#include "Eigen/Dense"

class Tools
{
  public:
    Tools() = default;
    ~Tools() = default;

    /// @brief A helper method to calculate RMSE.
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                         const std::vector<Eigen::VectorXd>& ground_truth);

    /// @brief A helper method to calculate Jacobians.
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
    static Eigen::MatrixXd CalculateJacobianStateTrasition(const Eigen::VectorXd& x_state);
};
