#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "main_utils.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void ReadLaserMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list)
{
    // LASER MEASUREMENT

    // read measurements at this timestamp
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float x;
    float y;
    long long timestamp;

    iss >> x;
    iss >> y;
    meas_package.raw_measurements_ << x, y;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
    measurement_pack_list.push_back(meas_package);
}

void ReadRadarMeasurement(MeasurementPackage& meas_package,
                          istringstream& iss,
                          vector<MeasurementPackage>& measurement_pack_list)
{
    // RADAR MEASUREMENT

    // read measurements at this timestamp
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    long long timestamp;

    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
    measurement_pack_list.push_back(meas_package);
}

int main(int argc, char* argv[])
{
    CheckArguments(argc, argv);

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    CheckFiles(in_file_, in_file_name_, out_file_, out_file_name_);

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;
    int kMaxMeasurement = 100;
    int counter = 0U;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line))
    {
        if (counter > kMaxMeasurement)
            break;
        counter++;

        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0)
        {
            ReadLaserMeasurement(meas_package, iss, measurement_pack_list);
        }
        else if (sensor_type.compare("R") == 0)
        {
            ReadRadarMeasurement(meas_package, iss, measurement_pack_list);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }

    // Create a Fusion EKF instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // Call the EKF-based fusion
    size_t N = measurement_pack_list.size();
    for (size_t k = 0; k < N; ++k)
    {
        // start filtering from the second frame (the speed is unknown in the first
        // frame)
        fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

        OutputEstimations(out_file_, fusionEKF, measurement_pack_list, k, gt_pack_list);

        estimations.push_back(fusionEKF.ekf_.x_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
    }

    // compute the accuracy (RMSE)
    Tools tools;
    std::cout << "Accuracy - RMSE: \n" << tools.CalculateRMSE(estimations, ground_truth) << std::endl;

    // close files
    if (out_file_.is_open())
    {
        out_file_.close();
    }

    if (in_file_.is_open())
    {
        in_file_.close();
    }

    return 0;
}
