#pragma once

#include "Eigen/Dense"

using Eigen::VectorXd;

class MeasurementPackage
{
  public:
    long long timestamp_;

    enum SensorType
    {
        LASER,
        RADAR
    } sensor_type_;

    VectorXd raw_measurements_;
};
