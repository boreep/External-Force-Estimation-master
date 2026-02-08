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
    void reset(double x0) { y = x0; initialized = true; }
    double update(double x, double dt) {
        if (!initialized) { reset(x); return x; }
        const double tau = 1.0 / (2.0 * M_PI * std::max(1e-6, fc));
        const double alpha = dt / (tau + dt);
        y = (1.0 - alpha) * y + alpha * x;
        return y;
    }
private:
    double fc; double y; bool initialized;
};

int main(int argc, char **argv) {
    // ================= 配置区域 =================
    const double START_TIME = 2.0;
    const double END_TIME   = 50.0;
    const double VEL_LP_FC_HZ = 10.0; // 速度滤波器频率

    string input_file  = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/q_ga_1_4_out_filterd.csv";
    string output_file = "/home/hxd/桌面/UR5-External-Force-Estimation-master/threshold_data.csv";
    if (argc > 1) input_file = argv[1];
    // ===========================================

    RM65 robot;
    auto observer = getObserver(&robot, 0, 0.005, "normal"); // 使用动量观测器或你想要的

    int dof = robot.jointNo();
    Vector q(dof), qd_raw(dof), qd_smooth(dof), tau_meas(dof);
    Vector qdd_est(dof); // 新增：估计的加速度
    Vector prev_qd_smooth = Vector::Zero(dof); // 用于差分
    Vector zero_vec = Vector::Zero(dof);

    vector<LowPass1st> vel_filters;
    for (int i = 0; i < dof; i++) vel_filters.emplace_back(VEL_LP_FC_HZ);

    ifstream ifs(input_file);
    if (!ifs.is_open()) { cerr << "Error opening " << input_file << endl; return -1; }
    ofstream ofs(output_file);

    // 输出表头：新增 m_i (inertia)
    ofs << "time";
    for(int i=0; i<dof; i++) ofs << ",res_" << i;
    for(int i=0; i<dof; i++) ofs << ",m_" << i;  // Inertia
    for(int i=0; i<dof; i++) ofs << ",g_" << i;  // Gravity
    for(int i=0; i<dof; i++) ofs << ",c_" << i;  // Coriolis
    for(int i=0; i<dof; i++) ofs << ",f_" << i;  // Friction
    ofs << endl;

    string line; getline(ifs, line); 

    double prev_time = 0.0;
    bool is_first_valid = true;

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

        double dt = 0.005;
        if (!is_first_valid) {
            dt = t - prev_time;
            if (dt <= 1e-5 || dt > 0.1) dt = 0.005;
        }

        // 滤波处理
        if (is_first_valid) {
            for (int i = 0; i < dof; i++) vel_filters[i].reset(qd_raw(i));
            qd_smooth = qd_raw;
            qdd_est.setZero();
            is_first_valid = false;
        } else {
            for (int i = 0; i < dof; i++) {
                qd_smooth(i) = vel_filters[i].update(qd_raw(i), dt);
            }
            // 关键：加速度由平滑速度差分得到，保证相位与观测器内部一致
            qdd_est = (qd_smooth - prev_qd_smooth) / dt;
        }
        prev_qd_smooth = qd_smooth;
        prev_time = t;

        // A. 观测器残差
        Vector residual = observer->getExternalTorque(q, qd_smooth, tau_meas, dt);

        // B. 动力学分量分解
        // 1. 惯性项: rnea(q, 0, qdd, 0)
        Vector tau_m = robot.rnea(q, zero_vec, qdd_est, 0.0);
        // 2. 重力项: rnea(q, 0, 0, g)
        Vector tau_g = robot.rnea(q, zero_vec, zero_vec, 9.81);
        // 3. 科氏力: rnea(q, qd, 0, 0)
        Vector tau_c = robot.rnea(q, qd_smooth, zero_vec, 0.0);
        // 4. 摩擦力
        Vector tau_f = robot.getFriction(qd_smooth);

        // 写 CSV
        ofs << t;
        for(int i=0; i<dof; i++) ofs << "," << residual(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_m(i); // 新增
        for(int i=0; i<dof; i++) ofs << "," << tau_g(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_c(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_f(i);
        ofs << endl;
    }

    cout << "Generated " << output_file << " with Inertia terms." << endl;
    return 0;
}