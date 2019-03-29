#include "benchmark.h"
using namespace benchmark;

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        fputs("Usage:\n  relocalization <groundtruth> <input> <has inertial>", stderr);
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

    auto shutter_times = read_shutter(dataset_root + "/shutter_time.csv");

    auto [gt_trajectory, in_trajectory] = get_synchronized_data(input_filename, vic_dataset, cam_dataset, imu_dataset);

    std::vector<double> recover_edges;
    if (!has_inertial) {
        for (size_t j = 1; j < in_trajectory.size(); ++j) {
            if (!is_valid_pose(in_trajectory[j - 1]) && is_valid_pose(in_trajectory[j])) {
                recover_edges.push_back(in_trajectory[j].t);
            }
        }

    } else {
        for (size_t j = 1; j < in_trajectory.size(); ++j) {
            if ((in_trajectory[j].p - in_trajectory[j - 1].p).norm() > 0.05) {
                recover_edges.push_back(in_trajectory[j].t);
            }
        }
    }

    double average_recover_time = 0;
    double average_recover_count = 0;
    for (size_t i = 0; i < shutter_times.size(); ++i) {
        for (size_t j = 0; j < recover_edges.size(); ++j) {
            if (recover_edges[j] > shutter_times[i].second) {
                double recover_time = recover_edges[j] - shutter_times[i].second;
                average_recover_time += recover_time;
                average_recover_count++;
                break;
            }
        }
    }
    average_recover_time /= std::max(average_recover_count, 1.0);
    printf("Relocalization Time: %.3f [s]", average_recover_time);

    return 0;
}
