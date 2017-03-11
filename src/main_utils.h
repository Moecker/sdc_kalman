#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

void OutputEstimations(ofstream& out_file_,
                       FusionEKF& fusionEKF,
                       vector<MeasurementPackage>& measurement_pack_list,
                       size_t k,
                       vector<GroundTruthPackage>& gt_pack_list)
{
    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
        // output the estimation
        out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
        out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    }
    else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
        // output the estimation in the cartesian coordinates
        double ro = measurement_pack_list[k].raw_measurements_(0);
        double phi = measurement_pack_list[k].raw_measurements_(1);
        out_file_ << ro * cos(phi) << "\t";  // p1_meas
        out_file_ << ro * sin(phi) << "\t";  // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";
}

void CheckArguments(int argc, char* argv[])
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1)
    {
        cerr << usage_instructions << endl;
    }
    else if (argc == 2)
    {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    }
    else if (argc == 3)
    {
        has_valid_args = true;
    }
    else if (argc > 3)
    {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args)
    {
        exit(EXIT_FAILURE);
    }
}

void CheckFiles(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name)
{
    if (!in_file.is_open())
    {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open())
    {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}