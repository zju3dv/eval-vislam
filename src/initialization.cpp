#include <deque>
#include "benchmark.h"
using namespace benchmark;

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        fputs("Usage:\n  initialization <groundtruth> <input> <has inertial>", stderr);
        return EXIT_FAILURE;
    }

    std::string dataset_root(argv[1]);
    std::string input_filename(argv[2]);
    bool has_inertial = false;

    if (argc == 3) {
        has_inertial = false;
    } else {
        has_inertial = (std::atoi(argv[3]) != 0);
    }

    CameraDataset cam_dataset;
    ImuDataset imu_dataset;
    ViconDataset vic_dataset;

    cam_dataset.load(dataset_root + "/camera");
    imu_dataset.load(dataset_root + "/imu");
    vic_dataset.load(dataset_root + "/groundtruth"); // gt has same format to vicon

    auto [gt_trajectory, in_trajectory] = get_synchronized_data(input_filename, vic_dataset, cam_dataset, imu_dataset);

    // find global scale s_g
    std::vector<vector<3>> gt_positions, in_positions;
    for (size_t i = 0; i < in_trajectory.size(); ++i) {
        if (is_valid_pose(in_trajectory[i])) {
            gt_positions.push_back(gt_trajectory[i].p);
            in_positions.push_back(in_trajectory[i].p);
        }
    }

    double s_g = 1.0;
    std::tie(s_g, std::ignore, std::ignore) = umeyama(gt_positions, in_positions);

    // find first consecutive sub-sequence
    size_t sequence_end = 0;
    while (sequence_end < in_trajectory.size() && is_valid_pose(in_trajectory[sequence_end])) {
        sequence_end++;
    }

    // compute the scale of cumulative windows
    std::deque<double> scales;
    for (size_t j = 1; j <= sequence_end; ++j) {
        if (j >= 5) {
            std::vector<vector<3>> gt_points, in_points;
            for (size_t i = 0; i < j; ++i) {
                gt_points.push_back(gt_trajectory[i].p);
                in_points.push_back(in_trajectory[i].p);
            }
            double s;
            std::tie(s, std::ignore, std::ignore) = umeyama(gt_points, in_points, has_inertial);
            scales.push_back(s);
        } else { // too little point and we cannot solve scale.
            scales.push_back(0);
        }
    }

    // find convergence point
    size_t convergence_point;
    for (convergence_point = sequence_end; convergence_point > 2; --convergence_point) {
        double r = abs(scales[convergence_point - 1] - scales.back()) / scales.back();
        if (r > 0.2) break;
    }

    double t_init = std::max(in_trajectory[convergence_point].t - vic_dataset.data.items[0].t - 5, 0.0);
    double r_scale = scales[convergence_point] / s_g;
    double error_scale = 0.5 * (abs(r_scale - 1) + abs(1 / r_scale - 1));

    double init_quality = t_init * sqrt(error_scale + 0.01);

    printf("T_init:  %.3f [s]\nE_scale: %f\nE_init:  %f\n", t_init, error_scale, init_quality);

    return EXIT_SUCCESS;
}
