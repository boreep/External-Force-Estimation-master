#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "RM65Simulation/RM65SimulationModel.h"
#include "RM65Simulation/observers.h"

using namespace std;

vector<string> parseCSV(const string &line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) result.push_back(item);
    return result;
}

// 一阶低通滤波器
class LowPass1st {
public:
    LowPass1st(double fc_hz = 10.0) : fc(fc_hz), y(0.0), initialized(false) {}

    void reset(double x0) {
        y = x0;
        initialized = true;
    }

    double update(double x, double dt) {
        if (!initialized) { reset(x); return x; }
        // tau = 1/(2*pi*fc)
        const double tau = 1.0 / (2.0 * M_PI * std::max(1e-6, fc));
        const double alpha = dt / (tau + dt);
        y = (1.0 - alpha) * y + alpha * x;
        return y;
    }

private:
    double fc;        // cutoff frequency (Hz)
    double y;
    bool initialized;
};

int main(int argc, char **argv) {
    // ================= 配置区域 =================
    const double START_TIME = 3.0;
    const double END_TIME   = 30.0;

    // 速度低通截止频率（Hz），建议 8~15 之间试
    const double VEL_LP_FC_HZ = 10.0;

    // 默认文件路径
    string input_file  = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/Traj3_out_formatted.csv";
    string output_file = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/threshold_data.csv";
    // ===========================================

    if (argc > 1) input_file = argv[1];

    cout << "Reading from: " << input_file << endl;
    cout << "Writing to:   " << output_file << endl;
    cout << "Time Range:   " << START_TIME << "s to " << END_TIME << "s" << endl;
    cout << "Vel LPF fc:   " << VEL_LP_FC_HZ << " Hz" << endl;

    RM65 robot;
    auto observer = getObserver(&robot, 5, 0.005, "real");

    int dof = robot.jointNo();
    Vector q(dof), qd_raw(dof), qd_smooth(dof), tau_meas(dof);
    Vector zero_vec = Vector::Zero(dof);

    // 为每个关节准备一个低通滤波器
    vector<LowPass1st> vel_filters;
    vel_filters.reserve(dof);
    for (int i = 0; i < dof; i++) vel_filters.emplace_back(VEL_LP_FC_HZ);

    ifstream ifs(input_file);
    if (!ifs.is_open()) {
        cerr << "Error: Cannot open input file " << input_file << endl;
        return -1;
    }

    ofstream ofs(output_file);
    if (!ofs.is_open()) {
        cerr << "Error: Cannot open output file " << output_file << endl;
        return -1;
    }

    // 输出表头：建议把 raw/smooth 也输出，方便你排查
    ofs << "time";
    for(int i=0; i<dof; i++) ofs << ",res_" << i;
    for(int i=0; i<dof; i++) ofs << ",g_" << i;
    for(int i=0; i<dof; i++) ofs << ",c_" << i;
    for(int i=0; i<dof; i++) ofs << ",f_" << i;
    for(int i=0; i<dof; i++) ofs << ",qd_raw_" << i;
    for(int i=0; i<dof; i++) ofs << ",qd_smooth_" << i;
    ofs << endl;

    string line;
    getline(ifs, line); // skip header

    double prev_time = 0.0;
    bool is_first_valid_sample = true;

    while (getline(ifs, line)) {
        vector<string> cols = parseCSV(line);
        if (cols.size() < 25) continue;

        double t = stod(cols[0]);

        if (t < START_TIME) continue;
        if (t > END_TIME) break;

        for(int i=0; i<dof; i++) {
            q(i)        = stod(cols[1 + i]);
            qd_raw(i)   = stod(cols[7 + i]);
            tau_meas(i) = stod(cols[19 + i]);
        }

        // dt 逻辑
        double dt = 0.005;
        if (is_first_valid_sample) {
            dt = 0.005;
        } else {
            dt = t - prev_time;
            if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        }

        // 对截取段第一帧做滤波器 reset，避免起始瞬态
        if (is_first_valid_sample) {
            for (int i = 0; i < dof; i++) vel_filters[i].reset(qd_raw(i));
            qd_smooth = qd_raw;
            is_first_valid_sample = false;
        } else {
            for (int i = 0; i < dof; i++) {
                qd_smooth(i) = vel_filters[i].update(qd_raw(i), dt);
            }
        }

        prev_time = t;

        // A. 观测器：用平滑速度
        Vector residual = observer->getExternalTorque(q, qd_smooth, tau_meas, dt);

        // B. 动力学分量：也用平滑速度保持一致
        Vector tau_g = robot.rnea(q, zero_vec, zero_vec, 9.81);
        Vector tau_c = robot.rnea(q, qd_smooth, zero_vec, 0.0);
        Vector tau_f = robot.getFriction(qd_smooth);

        // 写 CSV
        ofs << t;
        for(int i=0; i<dof; i++) ofs << "," << residual(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_g(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_c(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_f(i);
        for(int i=0; i<dof; i++) ofs << "," << qd_raw(i);
        for(int i=0; i<dof; i++) ofs << "," << qd_smooth(i);
        ofs << endl;
    }

    cout << "Processing complete. Data saved to " << output_file << endl;
    return 0;
}
