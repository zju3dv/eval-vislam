#include "benchmark.h"
using namespace benchmark;

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        fputs("Usage:\n  relocalization <groundtruth> <input> <has inertial> <jump detection>", stderr);
        return EXIT_FAILURE;
    }

    std::string dataset_root(argv[1]);
    std::string input_filename(argv[2]);
    const bool has_inertial = (std::atoi(argv[3]) != 0);

    const double jump_detection = (argc == 5) ? (std::atoi(argv[4])) : 0.05;

    CameraDataset cam_dataset;
    ImuDataset imu_dataset;
    ViconDataset vic_dataset;

    cam_dataset.load(dataset_root + "/camera");
    imu_dataset.load(dataset_root + "/imu");
    vic_dataset.load(dataset_root + "/groundtruth"); // gt has same format to vicon

    auto shutter_times = read_shutter(dataset_root + "/shutter_time.csv");

    auto [gt_trajectory, in_trajectory] = get_synchronized_data(input_filename, vic_dataset, cam_dataset, imu_dataset);

    double average_recover_time = 0;
    double average_recover_count = 0;

    if (!has_inertial) {
        // find next valid pose
        for (size_t i = 0; i < shutter_times.size(); ++i) {
            double black_frame_stop_time = shutter_times[i].second;
            for (size_t j = 0; j < in_trajectory.size(); ++j) {
                if (in_trajectory[j].t >= black_frame_stop_time && is_valid_pose(in_trajectory[j])) {
                    average_recover_time += in_trajectory[j].t - black_frame_stop_time;
                    average_recover_count++;
                    break;
                }
            }
        }
    } else {
        std::vector<double> recover_edges;
        for (size_t j = 1; j < in_trajectory.size(); ++j) {
            if ((in_trajectory[j].p - in_trajectory[j - 1].p).norm() > jump_detection) {
                recover_edges.push_back(in_trajectory[j].t);
            }
        }
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
    }

    average_recover_time /= std::max(average_recover_count, 1.0);

    const double sigma_reloc = has_inertial ? 1.42 : 0.65;
    if (average_recover_count > 0) {
        printf("Relocalization Time: %.3f [s] Score %.4f\n", average_recover_time, compute_score(average_recover_time, sigma_reloc));
    } else {
        printf("Relocalization not detected, Score 0\n");
    }

    return 0;
}
