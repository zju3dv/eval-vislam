#include "benchmark.h"
#include <cmath>
using namespace benchmark;

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fputs("Usage:\n  robustness <groundtruth> <input> <fix scale>", stderr);
        return EXIT_FAILURE;
    }

    std::string dataset_root(argv[1]);
    std::string input_filename(argv[2]);
    bool fix_scale = false;

    if (argc == 3) {
        fix_scale = false;
    } else {
        fix_scale = (std::atoi(argv[3]) != 0);
    }

    CameraDataset cam_dataset;
    ImuDataset imu_dataset;
    ViconDataset vic_dataset;

    cam_dataset.load(dataset_root + "/camera");
    imu_dataset.load(dataset_root + "/imu");
    vic_dataset.load(dataset_root + "/groundtruth"); // gt has same format to vicon

    auto [gt_trajectory, in_trajectory] = get_synchronized_data(input_filename, vic_dataset, cam_dataset, imu_dataset);

    std::vector<vector<3>> gt_positions, in_positions;
    for (size_t i = 0; i < in_trajectory.size(); ++i) {
        const PoseData &gt_pose = gt_trajectory[i];
        const PoseData &in_pose = in_trajectory[i];
        if (is_valid_pose(in_pose)) {
            gt_positions.push_back(gt_pose.p);
            in_positions.push_back(in_pose.p);
        }
    }
    auto [sg, qg, tg] = umeyama(gt_positions, in_positions, fix_scale);
    for (size_t i = 0; i < in_positions.size(); ++i) {
        in_positions[i] = (qg * in_positions[i] + tg) / sg;
    }
    double ape = 0, Acount = 0;
    for (size_t i = 0; i < in_positions.size(); ++i) {
        vector<3> p_error = in_positions[i] - gt_positions[i];
        ape += p_error.squaredNorm();
        Acount++;
    }
    Acount = std::max(Acount, 1.0);
    ape = sqrt(ape / Acount) * 1e3;

    size_t lost_count = 0;
    std::vector<std::vector<PoseData>> gt_segments, in_segments;
    gt_segments.emplace_back();
    in_segments.emplace_back();
    for (size_t i = 0; i < in_trajectory.size(); ++i) {
        const PoseData &gt_pose = gt_trajectory[i];
        const PoseData &in_pose = in_trajectory[i];
        if (is_valid_pose(in_pose)) {
            gt_segments.back().push_back(gt_pose);
            in_segments.back().push_back(in_pose);
        } else {
            lost_count++;
            if (!in_segments.back().empty()) {
                gt_segments.emplace_back();
                in_segments.emplace_back();
            }
        }
    }

    double lost_ratio = lost_count / (double)in_trajectory.size() * 1e2;

    std::vector<std::tuple<double, quaternion, vector<3>>> similarities;
    for (size_t i = 0; i < in_segments.size(); ++i) {
        const std::vector<PoseData> &gt_segment = gt_segments[i];
        const std::vector<PoseData> &in_segment = in_segments[i];
        std::vector<vector<3>> gt_positions, in_positions;
        for (size_t j = 0; j < in_segment.size(); ++j) {
            gt_positions.push_back(gt_segment[j].p);
            in_positions.push_back(in_segment[j].p);
        }
        similarities.emplace_back(umeyama(gt_positions, in_positions, fix_scale));
    }

    double relocalization_error = 0;
    for (size_t j = 1; j < similarities.size(); ++j) {
        auto Si = similarities[j - 1];
        auto Sj = similarities[j];
        double s_diff = std::get<0>(Si) - std::get<0>(Sj);
        if (std::isnan(s_diff)) continue; // not enough poses for umeyama
        vector<3> q_diff = logmap(std::get<1>(Si).conjugate() * std::get<1>(Sj));
        vector<3> t_diff = std::get<2>(Si) - std::get<2>(Sj);
        relocalization_error += sqrt(s_diff * s_diff + q_diff.squaredNorm() + t_diff.squaredNorm());
    }

    double robustness = (lost_ratio + 5) / 100.0 * (relocalization_error + 0.1 * ape);

    printf("lost ratio:  %.3f%%\nreloc error: %.3f\nAPE:         %.3f [mm]\nrobustness:  %.3f\n",
           lost_ratio,
           relocalization_error,
           ape,
           robustness);

    return EXIT_SUCCESS;
}
