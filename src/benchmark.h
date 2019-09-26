#ifndef BENCHMARK_DATASET_H
#define BENCHMARK_DATASET_H

#include <cstdio>
#include <string>
#include <fstream>
#include <algorithm>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

namespace benchmark {

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = double>
using vector = typename std::conditional<
    RowVector,
    matrix<1, Dimension, false, T>,
    matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

struct PoseData {
    double t;
    quaternion q;
    vector<3> p;
};

struct RunTimeData {
    double t;
    double duration;
};

struct ImuData {
    double t;
    vector<3> w;
    vector<3> a;
};

struct CameraData {
    double t;
    std::string filename;
};

using ViconData = PoseData;

struct ImuYaml {
    std::string type; // must be imu
    double frequency;
    struct {
        double sigma_w;
        double sigma_bw;
        double sigma_a;
        double sigma_ba;
    } intrinsic;
    struct {
        quaternion q;
        vector<3> p;
        double t;
    } extrinsic;

    void load(const std::string &filename) {
        YAML::Node node = YAML::LoadFile(filename);
        type = node["type"].as<std::string>();
        frequency = node["frequency"].as<double>();
        intrinsic.sigma_w = node["intrinsic"]["sigma_w"].as<double>();
        intrinsic.sigma_bw = node["intrinsic"]["sigma_bw"].as<double>();
        intrinsic.sigma_a = node["intrinsic"]["sigma_a"].as<double>();
        intrinsic.sigma_ba = node["intrinsic"]["sigma_ba"].as<double>();
        extrinsic.q.x() = node["extrinsic"]["q"][0].as<double>();
        extrinsic.q.y() = node["extrinsic"]["q"][1].as<double>();
        extrinsic.q.z() = node["extrinsic"]["q"][2].as<double>();
        extrinsic.q.w() = node["extrinsic"]["q"][3].as<double>();
        extrinsic.p.x() = node["extrinsic"]["p"][0].as<double>();
        extrinsic.p.y() = node["extrinsic"]["p"][1].as<double>();
        extrinsic.p.z() = node["extrinsic"]["p"][2].as<double>();
        extrinsic.t = node["extrinsic"]["t"].as<double>();
    }

    void save(const std::string &filename) const {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Comment("sense-slam sensor.yaml");
        emitter << YAML::Key << "type" << YAML::Value << type;
        emitter << YAML::Key << "frequency" << YAML::Value << frequency << YAML::Comment("[hz]");
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("noise parameters");
        emitter << YAML::Key << "intrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "sigma_w" << YAML::Value << intrinsic.sigma_w << YAML::Comment("[rad/s/sqrt(hz)]");
        emitter << YAML::Key << "sigma_bw" << YAML::Value << intrinsic.sigma_bw << YAML::Comment("[rad/s^2/sqrt(hz)]");
        emitter << YAML::Key << "sigma_a" << YAML::Value << intrinsic.sigma_a << YAML::Comment("[m/s^2/sqrt(hz)]");
        emitter << YAML::Key << "sigma_ba" << YAML::Value << intrinsic.sigma_ba << YAML::Comment("[m/s^3/sqrt(hz)]");
        emitter << YAML::EndMap;
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("sensor to body transform");
        emitter << YAML::Key << "extrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "q" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.q.x();
        emitter << extrinsic.q.y();
        emitter << extrinsic.q.z();
        emitter << extrinsic.q.w();
        emitter << YAML::EndSeq << YAML::Comment("x y z w");
        emitter << YAML::Key << "p" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.p.x();
        emitter << extrinsic.p.y();
        emitter << extrinsic.p.z();
        emitter << YAML::EndSeq << YAML::Comment("[m]");
        emitter << YAML::Key << "t" << YAML::Value << extrinsic.t << YAML::Comment("[s]");
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        emitter << YAML::Newline;
        std::ofstream(filename, std::ofstream::out)
            << "%YAML 1.0\n"
            << emitter.c_str();
    }
};

struct CameraYaml {
    std::string type; // must be camera
    double frequency;
    std::string camera_model;     // must be pinhole at now
    std::string distortion_model; // must be radtan at now
    struct {
        struct {
            double fx;
            double fy;
            double cx;
            double cy;
        } camera;
        struct {
            double k1;
            double k2;
            double r1;
            double r2;
        } distortion;
    } intrinsic;
    struct {
        quaternion q;
        vector<3> p;
        double t;
    } extrinsic;

    void load(const std::string &filename) {
        YAML::Node node = YAML::LoadFile(filename);
        type = node["type"].as<std::string>();
        frequency = node["frequency"].as<double>();
        camera_model = node["camera_model"].as<std::string>();
        distortion_model = node["distortion_model"].as<std::string>();
        intrinsic.camera.fx = node["intrinsic"]["camera"][0].as<double>();
        intrinsic.camera.fy = node["intrinsic"]["camera"][1].as<double>();
        intrinsic.camera.cx = node["intrinsic"]["camera"][2].as<double>();
        intrinsic.camera.cy = node["intrinsic"]["camera"][3].as<double>();
        intrinsic.distortion.k1 = node["intrinsic"]["distortion"][0].as<double>();
        intrinsic.distortion.k2 = node["intrinsic"]["distortion"][1].as<double>();
        intrinsic.distortion.r1 = node["intrinsic"]["distortion"][2].as<double>();
        intrinsic.distortion.r2 = node["intrinsic"]["distortion"][3].as<double>();
        extrinsic.q.x() = node["extrinsic"]["q"][0].as<double>();
        extrinsic.q.y() = node["extrinsic"]["q"][1].as<double>();
        extrinsic.q.z() = node["extrinsic"]["q"][2].as<double>();
        extrinsic.q.w() = node["extrinsic"]["q"][3].as<double>();
        extrinsic.p.x() = node["extrinsic"]["p"][0].as<double>();
        extrinsic.p.y() = node["extrinsic"]["p"][1].as<double>();
        extrinsic.p.z() = node["extrinsic"]["p"][2].as<double>();
        extrinsic.t = node["extrinsic"]["t"].as<double>();
    }

    void save(const std::string &filename) const {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Comment("sense-slam sensor.yaml");
        emitter << YAML::Key << "type" << YAML::Value << type;
        emitter << YAML::Key << "frequency" << YAML::Value << frequency << YAML::Comment("[hz]");
        emitter << YAML::Key << "camera_model" << YAML::Value << camera_model;
        emitter << YAML::Key << "distortion_model" << YAML::Value << distortion_model;
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("intrinsic parameters");
        emitter << YAML::Key << "intrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "camera" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << intrinsic.camera.fx;
        emitter << intrinsic.camera.fy;
        emitter << intrinsic.camera.cx;
        emitter << intrinsic.camera.cy;
        emitter << YAML::EndSeq << YAML::Comment("fx fy cx cy");
        emitter << YAML::Key << "distortion" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << intrinsic.distortion.k1;
        emitter << intrinsic.distortion.k2;
        emitter << intrinsic.distortion.r1;
        emitter << intrinsic.distortion.r2;
        emitter << YAML::EndSeq << YAML::Comment("k1 k2 r1 r2");
        emitter << YAML::EndMap;
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("sensor to body transform");
        emitter << YAML::Key << "extrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "q" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.q.x();
        emitter << extrinsic.q.y();
        emitter << extrinsic.q.z();
        emitter << extrinsic.q.w();
        emitter << YAML::EndSeq << YAML::Comment("x y z w");
        emitter << YAML::Key << "p" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.p.x();
        emitter << extrinsic.p.y();
        emitter << extrinsic.p.z();
        emitter << YAML::EndSeq << YAML::Comment("[m]");
        emitter << YAML::Key << "t" << YAML::Value << extrinsic.t << YAML::Comment("[s]");
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        emitter << YAML::Newline;
        std::ofstream(filename, std::ofstream::out)
            << "%YAML 1.0\n"
            << emitter.c_str();
    }
};

struct ViconYaml {
    std::string type; // must be vicon
    double frequency;
    struct {
        double sigma_q;
        double sigma_p;
    } intrinsic;
    struct {
        quaternion q;
        vector<3> p;
        double t;
    } extrinsic;

    void load(const std::string &filename) {
        YAML::Node node = YAML::LoadFile(filename);
        type = node["type"].as<std::string>();
        frequency = node["frequency"].as<double>();
        intrinsic.sigma_q = node["intrinsic"]["sigma_q"].as<double>();
        intrinsic.sigma_p = node["intrinsic"]["sigma_p"].as<double>();
        extrinsic.q.x() = node["extrinsic"]["q"][0].as<double>();
        extrinsic.q.y() = node["extrinsic"]["q"][1].as<double>();
        extrinsic.q.z() = node["extrinsic"]["q"][2].as<double>();
        extrinsic.q.w() = node["extrinsic"]["q"][3].as<double>();
        extrinsic.p.x() = node["extrinsic"]["p"][0].as<double>();
        extrinsic.p.y() = node["extrinsic"]["p"][1].as<double>();
        extrinsic.p.z() = node["extrinsic"]["p"][2].as<double>();
        extrinsic.t = node["extrinsic"]["t"].as<double>();
    }

    void save(const std::string &filename) const {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Comment("sense-slam sensor.yaml");
        emitter << YAML::Key << "type" << YAML::Value << type;
        emitter << YAML::Key << "frequency" << YAML::Value << frequency << YAML::Comment("[hz]");
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("noise parameters");
        emitter << YAML::Key << "intrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "sigma_q" << YAML::Value << intrinsic.sigma_q << YAML::Comment("[rad]");
        emitter << YAML::Key << "sigma_p" << YAML::Value << intrinsic.sigma_p << YAML::Comment("[m]");
        emitter << YAML::EndMap;
        emitter << YAML::Newline << YAML::Newline;
        emitter << YAML::Comment("sensor to body transform");
        emitter << YAML::Key << "extrinsic" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "q" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.q.x();
        emitter << extrinsic.q.y();
        emitter << extrinsic.q.z();
        emitter << extrinsic.q.w();
        emitter << YAML::EndSeq << YAML::Comment("x y z w");
        emitter << YAML::Key << "p" << YAML::Value;
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << extrinsic.p.x();
        emitter << extrinsic.p.y();
        emitter << extrinsic.p.z();
        emitter << YAML::EndSeq << YAML::Comment("[m]");
        emitter << YAML::Key << "t" << YAML::Value << extrinsic.t << YAML::Comment("[s]");
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        emitter << YAML::Newline;
        std::ofstream(filename, std::ofstream::out)
            << "%YAML 1.0\n"
            << emitter.c_str();
    }
};

struct ImuCsv {
    std::vector<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            fscanf(csv, "%*[^\r\n]");
            ImuData item;
            while (!feof(csv) && fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &item.t, &item.w.x(), &item.w.y(), &item.w.z(), &item.a.x(), &item.a.y(), &item.a.z()) == 7) {
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[s:double],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/s:double],a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/s^2:double]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%.14e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n", item.t, item.w.x(), item.w.y(), item.w.z(), item.a.x(), item.a.y(), item.a.z());
            }
            fclose(csv);
        }
    }
};

struct CameraCsv {
    std::vector<CameraData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            fscanf(csv, "%*[^\r\n]");
            char filename_buffer[2048];
            CameraData item;
            while (!feof(csv)) {
                memset(filename_buffer, 0, 2048);
                if (fscanf(csv, "%lf,%2047[^\n]\n", &item.t, filename_buffer) != 2) {
                    break;
                }
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[s:double],filename[string]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%.14e,%s\n", item.t, item.filename.c_str());
            }
            fclose(csv);
        }
    }
};

struct ViconCsv {
    std::vector<ViconData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            fscanf(csv, "%*[^\r\n]");
            ViconData item;
            while (!feof(csv) && fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &item.t, &item.q.x(), &item.q.y(), &item.q.z(), &item.q.w(), &item.p.x(), &item.p.y(), &item.p.z()) == 8) {
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[s:double],q.x[double],q.y[double],q.z[double],q.w[double],p.x[m:double],p.y[m:double],p.z[m:double]\n", csv);
            for (auto item : items) {
                fprintf(csv, "%.14e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n", item.t, item.q.x(), item.q.y(), item.q.z(), item.q.w(), item.p.x(), item.p.y(), item.p.z());
            }
            fclose(csv);
        }
    }
};

template <typename Yaml, typename Csv>
struct SensorDataset {
    Yaml sensor;
    Csv data;

    void load(const std::string &path) {
        sensor.load(path + "/sensor.yaml");
        data.load(path + "/data.csv");
    }

    void save(const std::string &path) const {
        sensor.save(path + "/sensor.yaml");
        data.save(path + "/data.csv");
    }
};

using ImuDataset = SensorDataset<ImuYaml, ImuCsv>;
using CameraDataset = SensorDataset<CameraYaml, CameraCsv>;
using ViconDataset = SensorDataset<ViconYaml, ViconCsv>;

inline bool is_valid_pose(const PoseData &pose) {
    if (abs(pose.q.norm() - 1.0) > 1.0e-3) {
        return false;
    }
    if (pose.p.x() == 0 && pose.p.y() == 0 && pose.p.z() == 0) {
        return false;
    }
    if (pose.p.x() == 0 && pose.p.y() == 0 && pose.p.z() == 0 && pose.q.x() == 0 && pose.q.y() == 0 && pose.q.z() == 0 && pose.q.w() == 1) {
        return false;
    }
    return true;
}

inline std::vector<std::pair<double, double>> read_shutter(const std::string &shutter_path) {
    std::vector<std::pair<double, double>> shutter_data;

    FILE *shutter_file = fopen(shutter_path.c_str(), "r");

    fscanf(shutter_file, "%*[^\r\n]");

    double begin_time, end_time;
    while (!feof(shutter_file)) {
        int f;
        double t;
        if (fscanf(shutter_file, "%d,%lf\n", &f, &t) == 0) {
            break;
        }
        if (f == 1) {
            begin_time = t;
        } else {
            end_time = t;
            shutter_data.emplace_back(begin_time, end_time);
        }
    }

    fclose(shutter_file);

    return shutter_data;
}

inline PoseData sample_pose(const std::vector<PoseData> &poses, double t, double *sample_t_diff = nullptr) {
    if (t <= poses.front().t) {
        if (sample_t_diff) *sample_t_diff = std::abs(t - poses.front().t);
        return poses.front();
    }
    if (t >= poses.back().t) {
        if (sample_t_diff) *sample_t_diff = std::abs(t - poses.back().t);
        return poses.back();
    }

    size_t l = 0, r = poses.size();
    while (l + 1 < r) {
        size_t m = l + (r - l) / 2;
        if (poses[m].t < t) {
            l = m;
        } else if (poses[m].t > t) {
            r = m;
        } else {
            l = r = m;
        }
    }

    if (sample_t_diff) *sample_t_diff = std::max(std::abs(t - poses[l].t), std::abs(t - poses[r].t));
    if (l == r) {
        return poses[l];
    }

    double lambda = (t - poses[l].t) / (poses[r].t - poses[l].t);
    PoseData pose;
    pose.t = t;
    pose.q = poses[l].q.slerp(lambda, poses[r].q);
    pose.p = poses[l].p + lambda * (poses[r].p - poses[l].p);
    return pose;
}

inline matrix<3> kabsch(const std::vector<vector<3>> &src, const std::vector<vector<3>> &dst) {
    matrix<3> cov = matrix<3>::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        cov += src[i] * dst[i].transpose();
    }
    cov = cov * (1.0 / src.size());
    Eigen::JacobiSVD<matrix<3>> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const matrix<3> &U = svd.matrixU();
    const matrix<3> &V = svd.matrixV();
    matrix<3> E = matrix<3>::Identity();
    if ((V * U.transpose()).determinant() >= 0.0) {
        E(2, 2) = 1.0;
    } else {
        E(2, 2) = -1.0;
    }

    return V * E * U.transpose();
}

inline std::tuple<double, quaternion, vector<3>> umeyama(std::vector<vector<3>> gt, std::vector<vector<3>> in, bool fix_scale = false) {
    vector<3> gt_avg = vector<3>::Zero();
    vector<3> in_avg = vector<3>::Zero();
    for (size_t i = 0; i < gt.size(); ++i) {
        gt_avg += gt[i];
        in_avg += in[i];
    }
    gt_avg /= (double)gt.size();
    in_avg /= (double)in.size();

    double gt_d2 = 0;
    double in_d2 = 0;
    for (size_t i = 0; i < gt.size(); ++i) {
        gt[i] -= gt_avg;
        in[i] -= in_avg;
        gt_d2 += gt[i].squaredNorm();
        in_d2 += in[i].squaredNorm();
    }

    double S = sqrt(in_d2 / gt_d2);
    if (fix_scale) {
        S = 1;
    }
    matrix<3> R = kabsch(in, gt);
    vector<3> T = S * gt_avg - R * in_avg;

    quaternion q;
    q = R;

    return {S, q, T};
}

inline vector<3> logmap(const quaternion &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

inline std::vector<PoseData> read_euroc_groundtruth(const std::string &filepath) {
    std::vector<PoseData> euroc_data;

    FILE *euroc_file = fopen(filepath.c_str(), "r");

    char header[1024];
    fscanf(euroc_file, "%[^\n]\n", header);

    while (!feof(euroc_file)) {
        PoseData data;
        if (fscanf(euroc_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf,%*lf\n", &data.t, &data.p.x(), &data.p.y(), &data.p.z(), &data.q.w(), &data.q.x(), &data.q.y(), &data.q.z()) == 0) {
            break;
        }
        data.t /= 1000000000;
        euroc_data.push_back(data);
    }

    fclose(euroc_file);

    return euroc_data;
}

inline std::vector<PoseData> read_tum_input(const std::string &tum_path, const CameraYaml &camera_yaml, const ImuYaml &imu_yaml) {
    std::vector<PoseData> tum_data;

    const quaternion &q_bi = imu_yaml.extrinsic.q;
    const vector<3> &p_bi = imu_yaml.extrinsic.p;
    const quaternion &q_bc = camera_yaml.extrinsic.q;
    const vector<3> &p_bc = camera_yaml.extrinsic.p;

    FILE *tum_file = fopen(tum_path.c_str(), "r");
    if (!tum_file) {
        fprintf(stderr, "Cannot open trajectory file : %s.\n", tum_path.c_str());
        exit(-1);
    }

    while (!feof(tum_file)) {
        PoseData data;
        if (fscanf(tum_file, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &data.t, &data.p.x(), &data.p.y(), &data.p.z(), &data.q.x(), &data.q.y(), &data.q.z(), &data.q.w()) == 0) {
            break;
        }
        PoseData imu_data;
        imu_data.t = data.t;
        if (is_valid_pose(data)) {
            imu_data.q = data.q * q_bc.conjugate() * q_bi;
            imu_data.p = data.p + data.q * q_bc.conjugate() * (p_bi - p_bc);
        } else {
            imu_data.q.x() = imu_data.q.y() = imu_data.q.z() = imu_data.q.w() = 0;
            imu_data.p.x() = imu_data.p.y() = imu_data.p.z() = 0;
        }
        tum_data.push_back(imu_data);
    }

    fclose(tum_file);

    return tum_data;
}

inline std::vector<RunTimeData> read_run_time(const std::string &time_path) {
    std::vector<RunTimeData> run_time_data;

    FILE *time_file = fopen(time_path.c_str(), "r");
    if (!time_file) {
        fprintf(stderr, "Cannot open time file : %s.\n", time_path.c_str());
        exit(-1);
    }
    double last_time = -1;
    while (!feof(time_file)) {
        RunTimeData data;
        if (fscanf(time_file, "%lf %lf\n", &data.t, &data.duration) == 0) {
            break;
        }
        if (last_time < 0) {
            last_time = data.duration;
        } else {
            double current_time = data.duration;
            data.duration = data.duration - last_time;
            last_time = current_time;
            run_time_data.push_back(data);
        }
    }

    fclose(time_file);

    return run_time_data;
}

inline std::pair<std::vector<PoseData>, std::vector<PoseData>> sample_groundtruth(const ViconDataset &vicon_dataset, const std::vector<PoseData> &in_poses, const ImuYaml &imu_yaml) {
    const quaternion &q_bi = imu_yaml.extrinsic.q;
    const vector<3> &p_bi = imu_yaml.extrinsic.p;
    const quaternion &q_bv = vicon_dataset.sensor.extrinsic.q;
    const vector<3> &p_bv = vicon_dataset.sensor.extrinsic.p;

    std::vector<PoseData> aligned_in_trajectory, groundtruth_trajectory;
    for (const PoseData &in_pose : in_poses) {
        double sample_t_diff;
        PoseData pose = sample_pose(vicon_dataset.data.items, in_pose.t, &sample_t_diff);
        if (sample_t_diff > 0.1) continue;
        PoseData gt_pose;
        gt_pose.t = pose.t + vicon_dataset.sensor.extrinsic.t;
        gt_pose.q = pose.q * q_bv.conjugate() * q_bi;
        gt_pose.p = pose.p + pose.q * q_bv.conjugate() * (p_bi - p_bv);
        groundtruth_trajectory.emplace_back(gt_pose);
        aligned_in_trajectory.emplace_back(in_pose);
    }
    return {aligned_in_trajectory, groundtruth_trajectory};
}

inline std::pair<std::vector<PoseData>, std::vector<PoseData>> get_synchronized_data(const std::string &input_filename, const ViconDataset &gt_dataset, const CameraDataset &cam_dataset, const ImuDataset &imu_dataset, size_t *in_trajectory_raw_valid_size = nullptr) {
    if (gt_dataset.data.items.size() == 0) {
        fputs("Groundtruth is empty.", stderr);
        exit(EXIT_FAILURE);
    }

    double gt_time_min = gt_dataset.data.items.front().t;
    double gt_time_max = gt_dataset.data.items.back().t;

    if (gt_time_max <= gt_time_min) {
        fputs("Groundtruth is empty.", stderr);
        exit(EXIT_FAILURE);
    }

    std::vector<PoseData> in_trajectory = read_tum_input(input_filename, cam_dataset.sensor, imu_dataset.sensor);

    // remove starting invalid poses, these poses means not initialized.
    while (in_trajectory.size() > 0 && !is_valid_pose(in_trajectory.front())) {
        in_trajectory.erase(in_trajectory.begin());
    }
    if (in_trajectory_raw_valid_size) *in_trajectory_raw_valid_size = in_trajectory.size();

    if (in_trajectory.size() == 0) {
        fputs("Input is empty.", stderr);
        exit(EXIT_FAILURE);
    }

    double in_time_min = in_trajectory.front().t;
    double in_time_max = in_trajectory.back().t;

    double overlap_time_min = std::max(gt_time_min, in_time_min);
    double overlap_time_max = std::min(gt_time_max, in_time_max);
    double overlap_time_len = overlap_time_max - overlap_time_min;

    if (overlap_time_len / (in_time_max - in_time_min) < 0.1) {
        fputs("Input time range does not match groundtruth.\n", stderr);
        exit(EXIT_FAILURE);
    }

    auto [sync_in_trajectory, gt_trajectory] = sample_groundtruth(gt_dataset, in_trajectory, imu_dataset.sensor);
    return std::make_pair(gt_trajectory, sync_in_trajectory);
}

inline std::vector<RunTimeData> get_synchronized_run_time_data(const std::string &time_filename, const std::vector<PoseData> &pose_data) {
    auto run_time_data = read_run_time(time_filename);
    std::vector<RunTimeData> sync_run_time_data;
    for (const auto &pose : pose_data) {
        RunTimeData d;
        d.t = pose.t;
        auto it = std::upper_bound(run_time_data.begin(), run_time_data.end(), d,
                                   [](const RunTimeData &lhs, const RunTimeData &rhs) -> bool { return lhs.t < rhs.t; });
        // first member is closest
        if (it == run_time_data.begin()) {
            sync_run_time_data.push_back(*it);
        } else if (it == run_time_data.end()) {
            sync_run_time_data.push_back(*(it - 1));
        } else {
            double diff1 = abs(pose.t - it->t);
            double diff2 = abs(pose.t - (it - 1)->t);
            if (diff2 < diff1)
                --it;
            sync_run_time_data.push_back(*it);
        }
    }
    return sync_run_time_data;
}

inline double compute_score(const double &val, const double &sigma) {
    return (sigma * sigma) / (sigma * sigma + val * val);
}

} // namespace benchmark

#endif // BENCHMARK_DATASET_H
