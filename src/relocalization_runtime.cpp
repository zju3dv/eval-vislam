#include "benchmark.h"
using namespace benchmark;

int main(int argc, char* argv[]) {
    if (argc < 5 || argc > 7) {
        fputs("Usage:\n  relocalization <groundtruth> <input-pose> <input-time> <has inertial> <jump detection> <time ratio>", stderr);
        return EXIT_FAILURE;
    }

    std::string dataset_root(argv[1]);
    std::string input_pose_filename(argv[2]);
    std::string input_time_filename(argv[3]);
    const bool has_inertial = (std::atoi(argv[4]) != 0);

    const double jump_detection = (argc >= 6) ? (std::atof(argv[5])) : 0.05;
    const double time_ratio = (argc >= 7) ? (std::atof(argv[6])) : 1.0;

    CameraDataset cam_dataset;
    ImuDataset imu_dataset;
    ViconDataset vic_dataset;

    cam_dataset.load(dataset_root + "/camera");
    imu_dataset.load(dataset_root + "/imu");
    vic_dataset.load(dataset_root + "/groundtruth"); // gt has same format to vicon

    auto shutter_times = read_shutter(dataset_root + "/shutter_time.csv");

    auto [gt_trajectory, in_trajectory] = get_synchronized_data(input_pose_filename, vic_dataset, cam_dataset, imu_dataset);
    auto run_time = get_synchronized_run_time_data(input_time_filename, in_trajectory);

    double average_recover_time = 0;
    double average_recover_count = 0;

    // find next valid pose
    for (size_t i = 0; i < shutter_times.size(); ++i) {
        double black_frame_stop_time = shutter_times[i].second;
        double next_black_frame_start_time = i == shutter_times.size() - 1 ? in_trajectory.back().t : shutter_times[i + 1].first;
        double current_recover_time = 0;
        bool find_recover_edge = false;
        for (size_t j = 1; j < in_trajectory.size(); ++j) {
            if (in_trajectory[j].t >= black_frame_stop_time && in_trajectory[j].t < next_black_frame_start_time) {
                current_recover_time += std::max(run_time[j].duration * time_ratio, 1.0 / 30);
                if (has_inertial) {
                    find_recover_edge = (in_trajectory[j].p - in_trajectory[j - 1].p).norm() > jump_detection;
                } else {
                    find_recover_edge = is_valid_pose(in_trajectory[j]);
                }
                if (find_recover_edge) {
                    printf("Find Recover edge at %.12e, black frame stop %.12e\n", in_trajectory[j].t, black_frame_stop_time);
                    average_recover_time += current_recover_time;
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
