#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "RM65Simulation/RM65SimulationModel.h"
#include "RM65Simulation/observers.h" 
#include "RM65Simulation/TrackingDifferentiator.h"

using namespace std;

// 简单的限幅滤波器 (Rate Limiter)
// 用于限制加速度的瞬间变化率 (即 Jerk 限制)
class RateLimiter {
private:
    double last_val;
    double max_rate; // 允许的最大单步变化量
    bool initialized;
public:
    RateLimiter(double limit) : max_rate(limit), last_val(0), initialized(false) {}
    
    void reset(double val) {
        last_val = val;
        initialized = true;
    }

    double update(double input) {
        if (!initialized) { reset(input); return input; }
        
        double diff = input - last_val;
        // 削峰逻辑
        if (diff > max_rate) input = last_val + max_rate;
        else if (diff < -max_rate) input = last_val - max_rate;
        
        last_val = input;
        return input;
    }
};

vector<string> parseCSV(const string &line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) {
        result.push_back(item);
    }
    return result;
}

int main(int argc, char **argv) {
    // ================= 配置区域 =================
    string input_file = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/Traj3_out_formatted.csv"; 
    string output_features_file = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/threshold_data_full.csv"; 
    string output_traj_file = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/Traj3_out_formatted_filtered.csv";

    // 1. 第一级 TD (速度 -> 加速度)
    // 负责从噪声速度中提取微分。r 稍微大一点以保证动态响应。
    double td1_r = 3000.0;     
    double td1_h = 0.02;       

    // 2. 限制器 (去除加速度毛刺)
    // 允许相邻两个周期(5ms)加速度最大变化量。
    // 假设最大 Jerk = 500 rad/s^3, dt=0.005 -> max_diff = 2.5
    double acc_change_limit = 0.1; 

    // 3. 第二级 TD (加速度 -> 平滑加速度)
    // 负责平滑加速度。r 小一点以滤除噪声。
    double td2_r = 500.0;    
    double td2_h = 0.02;      
    // ===========================================

    if (argc > 1) input_file = argv[1];

    RM65 robot;
    auto observer = getObserver(&robot, 5, 0.005, "real"); 
    int dof = robot.jointNo();

    // 初始化两级滤波器
    vector<TrackingDifferentiator> tds_vel; // 第一级
    vector<TrackingDifferentiator> tds_acc; // 第二级
    vector<RateLimiter> limiters;           // 中间限幅

    for(int i=0; i<dof; i++) {
        tds_vel.emplace_back(td1_r, td1_h);
        tds_acc.emplace_back(td2_r, td2_h);
        limiters.emplace_back(acc_change_limit);
    }

    ifstream ifs(input_file);
    ofstream ofs_feat(output_features_file); 
    ofstream ofs_traj(output_traj_file);     

    // 写入表头
    ofs_feat << "time";
    for(int i=0; i<dof; i++) ofs_feat << ",res_" << i;
    for(int i=0; i<dof; i++) ofs_feat << ",g_" << i;
    for(int i=0; i<dof; i++) ofs_feat << ",c_" << i;
    for(int i=0; i<dof; i++) ofs_feat << ",f_" << i;
    for(int i=0; i<dof; i++) ofs_feat << ",m_" << i;
    ofs_feat << endl;

    ofs_traj << "timestamp";
    for(int i=1; i<=6; i++) ofs_traj << ",pos" << i;
    for(int i=1; i<=6; i++) ofs_traj << ",vel" << i;
    for(int i=1; i<=6; i++) ofs_traj << ",acc" << i;
    for(int i=1; i<=6; i++) ofs_traj << ",torque" << i;
    ofs_traj << endl;

    string line;
    getline(ifs, line); 

    Vector q(dof), qd_raw(dof), tau_meas(dof);
    Vector qd_smooth(dof);  
    Vector qdd_rough(dof);   
    Vector qdd_limited(dof);
    Vector qdd_final(dof);   
    Vector zero_vec = Vector::Zero(dof);

    double prev_time = 0;
    bool first_line = true;

    while (getline(ifs, line)) {
        vector<string> cols = parseCSV(line);
        if (cols.size() < 25) continue; 

        double t = stod(cols[0]);
        for(int i=0; i<dof; i++) {
            q(i) = stod(cols[1 + i]);
            qd_raw(i) = stod(cols[7 + i]); 
            tau_meas(i) = stod(cols[19 + i]);
        }

        double dt = 0.005;
        if (!first_line) {
            dt = t - prev_time;
            if(dt <= 0 || dt > 0.1) dt = 0.005;
        }

        // ============================================
        // 核心滤波管道 (Pipeline)
        // ============================================
        for(int i=0; i<dof; i++) {
            if (first_line) {
                tds_vel[i].reset(qd_raw(i));
                limiters[i].reset(0.0);
                tds_acc[i].reset(0.0);
            }

            // 1. 第一级 TD: 速度 -> 粗加速度
            tds_vel[i].update(qd_raw(i), dt);
            qd_smooth(i) = tds_vel[i].getSmoothedVelocity();
            qdd_rough(i) = tds_vel[i].getAcceleration();

            // 2. 限幅器: 粗加速度 -> 去毛刺加速度
            // 如果加速度瞬间跳变太大，这步会把它拉回来
            qdd_limited(i) = limiters[i].update(qdd_rough(i));

            // 3. 第二级 TD: 加速度 -> 平滑加速度
            // 这里把加速度当作"位置"信号喂给 TD，取其跟踪输出(x1)
            tds_acc[i].update(qdd_limited(i), dt);
            qdd_final(i) = tds_acc[i].getSmoothedVelocity(); 
        }

        // --- 动力学重算 ---
        Vector residual = observer->getExternalTorque(q, qd_smooth, tau_meas, dt);
        Vector tau_g = robot.rnea(q, zero_vec, zero_vec, 9.8);
        Vector tau_c = robot.rnea(q, qd_smooth, zero_vec, 0.0);
        Vector tau_f = robot.getFriction(qd_smooth);
        Vector tau_m = robot.rnea(q, zero_vec, qdd_final, 0.0); // 使用双重滤波后的加速度

        // --- 保存文件 ---
        ofs_feat << t;
        for(int i=0; i<dof; i++) ofs_feat << "," << residual(i);
        for(int i=0; i<dof; i++) ofs_feat << "," << tau_g(i);
        for(int i=0; i<dof; i++) ofs_feat << "," << tau_c(i);
        for(int i=0; i<dof; i++) ofs_feat << "," << tau_f(i);
        for(int i=0; i<dof; i++) ofs_feat << "," << tau_m(i);
        ofs_feat << endl;

        ofs_traj << t;
        for(int i=0; i<dof; i++) ofs_traj << "," << q(i);
        for(int i=0; i<dof; i++) ofs_traj << "," << qd_smooth(i); // 平滑速度
        for(int i=0; i<dof; i++) ofs_traj << "," << qdd_final(i); // 最终加速度
        for(int i=0; i<dof; i++) ofs_traj << "," << tau_meas(i);
        ofs_traj << endl;

        prev_time = t;
        first_line = false;
    }

    cout << "Double-TD Filtering Complete!" << endl;
    cout << "Filtered Trajectory: " << output_traj_file << endl;
    
    return 0;
}