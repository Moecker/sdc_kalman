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
    const int kAll = 100000;
    const int kMaxMeasurement = kAll;
    int counter = 0U;

    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line))
    {
        if (counter >= kMaxMeasurement)
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

        ReadGroundTruth(iss, gt_package, gt_pack_list);
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
        std::cout << "Cycle: " << k + 1 << std::endl;
        // start filtering from the second frame (the speed is unknown in the first
        // frame)
        fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

        // Write estimations to output file
        OutputEstimations(out_file_, fusionEKF, measurement_pack_list, k, gt_pack_list);

        // Store ground truth and current Kalman state
        estimations.push_back(fusionEKF.ekf_.state_x_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
    }

    // Compute the accuracy (RMSE)
    std::cout << "Accuracy - RMSE: \n" << Tools::CalculateRMSE(estimations, ground_truth) << std::endl;

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
