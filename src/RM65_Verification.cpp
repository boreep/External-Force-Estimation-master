// src/RM65_Verification.cpp
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <cmath>

// 引入 RM65 模型头文件
#include "RM65Simulation/RM65SimulationModel.h"
#include "gnuplot-iostream.h"

using namespace std;

// 简单的 CSV 解析辅助函数
vector<string> split(const string &s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char **argv) {
    // 1. 设置输入文件路径 (可以直接硬编码或者通过参数传入)
    // 假设 CSV 文件就在当前运行目录下
    string inputFile = "/home/hxd/桌面/UR5-External-Force-Estimation-master/rmrobot/q_ga_5_out_processed.csv";
    if (argc > 1) {
        inputFile = argv[1];
    }

    // 2. 初始化 RM65 机器人模型
    // RM65 的构造函数会自动加载 rm65_config_data.h 中的参数
    RM65 robot;
    int dof = robot.jointNo(); // 6

    // 3. 准备数据容器
    Vector q(dof), qd(dof), q2d(dof);
    Vector tau_model(dof), tau_meas(dof);
    
    // 用于画图的数据: vector<tuple<time, value>>
    vector<vector<tuple<double, double>>> plot_meas(dof);
    vector<vector<tuple<double, double>>> plot_calc(dof);

    // 4. 读取 CSV 文件
    ifstream ifs(inputFile);
    if (!ifs.is_open()) {
        cerr << "Error: Cannot open input file: " << inputFile << endl;
        return -1;
    }

    string line;
    // 跳过表头: timestamp, pos1..6, vel1..6, acc1..6, torque1..6
    getline(ifs, line); 

    int row_count = 0;
    double timestamp = 0.0;

    cout << "Start processing data from " << inputFile << "..." << endl;

    while (getline(ifs, line)) {
        if (line.empty()) continue;
        
        vector<string> cols = split(line, ',');
        // 检查列数是否足够 (1 + 6*4 = 25 列)
        if (cols.size() < 25) continue;

        // 解析数据
        // index 0: timestamp
        timestamp = stod(cols[0]);

        // index 1-6: pos
        // index 7-12: vel
        // index 13-18: acc (使用你之前滤波处理好的加速度)
        // index 19-24: torque (measured)
        for (int i = 0; i < dof; ++i) {
            q(i)   = stod(cols[1 + i]);
            qd(i)  = stod(cols[7 + i]);
            q2d(i) = stod(cols[13 + i]); 
            tau_meas(i) = stod(cols[19 + i]);
        }

        // 5. 调用核心动力学算法
        // 计算刚体动力学力矩 (M*qdd + C*qd + G)
        Vector rnea_tau = robot.rnea(q, qd, q2d, 0); 
        
        // 计算摩擦力矩
        Vector fric_tau = robot.getFriction(qd);

        // 总理论力矩
        Vector total_tau = rnea_tau + fric_tau;

        // 6. 保存画图数据
        for (int i = 0; i < dof; ++i) {
            plot_meas[i].push_back(make_tuple(timestamp, tau_meas(i)));
            plot_calc[i].push_back(make_tuple(timestamp, total_tau(i)));
        }
        
        row_count++;
    }
    ifs.close();
    cout << "Processed " << row_count << " rows." << endl;

    // 7. 使用 Gnuplot 画图
    Gnuplot gp;
    gp << "set term qt size 1600, 900 title 'RM65 Dynamics Verification'\n"; // 使用 qt 窗口显示
    gp << "set multiplot layout 2,3 columnsfirst title 'Measured (Blue) vs Calculated (Red) Torque'\n";
    gp << "set grid\n";

    for (int i = 0; i < dof; ++i) {
        gp << "set title 'Joint " << (i + 1) << "'\n";
        gp << "set xlabel 'Time (s)'\n";
        gp << "set ylabel 'Torque (Nm)'\n";
        // 设置范围自动调整，或者根据需要固定
        // gp << "set yrange [-50:50]\n"; 

        gp << "plot '-' with lines title 'Measured' lc rgb 'blue' lw 1.5, "
           << "'-' with lines title 'Calculated' lc rgb 'red' lw 1.5\n";
        
        gp.send1d(plot_meas[i]);
        gp.send1d(plot_calc[i]);
    }

    gp << "unset multiplot\n";
    
    // 等待用户按回车退出，否则窗口会一闪而过
    cout << "Press Enter to exit..." << endl;
    cin.get();

    return 0;
}