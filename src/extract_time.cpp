#include <cstdio>
#include <iostream>
#include <string>
#include "benchmark.h"

using namespace benchmark;

inline bool is_file_exist(const std::string &name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
    std::string dataset_root(argv[1]);
    const std::string time_filename(argv[2]);
    const std::string nn_time_filename = time_filename.substr(0, time_filename.length() - 4) + "-nn.txt";

    CameraDataset cam_dataset;
    cam_dataset.load(dataset_root + "/camera");

    auto run_time = read_run_time(time_filename, true);
    
    size_t count_frame_num = 0;
    for (const auto &d : cam_dataset.data.items) {
        if (d.t < run_time.front().t) continue;
        if (d.t > run_time.back().t) continue;
        count_frame_num++;
    }

    double slam_run_time = run_time.back().duration - run_time.front().duration;

    if (is_file_exist(nn_time_filename)) {
        auto nn_run_time = read_run_time(nn_time_filename, true);
        double nn_runtime_seconds = nn_run_time.back().duration - nn_run_time.front().duration;
        // if runtime is not accurate enouch (close to int), we add 1 second
        const double EPS = 1e-6;
        if (std::abs(nn_runtime_seconds - int(nn_runtime_seconds)) < EPS) {
            nn_runtime_seconds += 1.0;
        }
        slam_run_time += nn_runtime_seconds;
    }

    printf("slam_run_count: %zu\nslam_run_time: %.3f\n", count_frame_num, slam_run_time);

    return 0;
}
