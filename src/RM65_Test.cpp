#include <boost/program_options.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <sstream>

#include "RM65Simulation/RM65SimulationModel.h"
#include "RM65Simulation/observers.h"
#include "RM65Simulation/DynamicThresholdDetector.h"
#include "gnuplot-iostream.h"

using namespace std;
namespace fs = std::filesystem;

// CSV 解析
vector<string> parseCSVLine(const string &line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) result.push_back(item);
    return result;
}

// ============================================================
// 一阶低通滤波器
// ============================================================
class LowPass1st {
public:
    explicit LowPass1st(double fc_hz = 10.0) : fc(fc_hz), y(0.0), initialized(false) {}
    void reset(double x0) { y = x0; initialized = true; }
    void setFc(double fc_hz) { fc = fc_hz; }
    double update(double x, double dt) {
        if (!initialized) { reset(x); return x; }
        const double tau = 1.0 / (2.0 * M_PI * std::max(1e-6, fc));
        const double a = dt / (tau + dt);
        y = (1.0 - a) * y + a * x;
        return y;
    }
private:
    double fc; double y; bool initialized;
};

int main(int argc, char **argv) {
    // ---------------------------------------------------------
    // 参数解析
    // ---------------------------------------------------------
    boost::program_options::options_description description("Allowed options");
    description.add_options()
        ("help,h", "Help message")
        ("inputFile,i", boost::program_options::value<string>()->default_value("Traj3_out_formatted.csv"), "input csv")
        ("outputFile,o", boost::program_options::value<string>()->default_value("test_result.csv"), "output csv")
        ("startTime,s", boost::program_options::value<double>()->default_value(2.0), "Start time")
        ("endTime,e", boost::program_options::value<double>()->default_value(30.0), "End time")
        ("addSimForce,f", boost::program_options::value<bool>()->default_value(true), "Add sim force?")
        ("simAmp,a", boost::program_options::value<double>()->default_value(5.0), "Sim Amp")
        ("simFreq,w", boost::program_options::value<double>()->default_value(0.25), "Sim Freq")
        ("simStart,S", boost::program_options::value<double>()->default_value(5.0), "Sim Start Time")
        ("velFc", boost::program_options::value<double>()->default_value(10.0), "Velocity LPF Hz")
        ("measFc", boost::program_options::value<double>()->default_value(30.0), "Torque LPF Hz");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) { cout << description << endl; return 0; }

    string inputFileName = vm["inputFile"].as<string>();
    string outputFileName = vm["outputFile"].as<string>();
    double startTime = vm["startTime"].as<double>();
    double endTime = vm["endTime"].as<double>();
    bool addSimForce = vm["addSimForce"].as<bool>();
    double simAmp = vm["simAmp"].as<double>();
    double simFreq = vm["simFreq"].as<double>();
    double simStartTime = vm["simStart"].as<double>();
    double velFc = vm["velFc"].as<double>();
    double measFc = vm["measFc"].as<double>();

    // 路径处理 (根据你的实际情况调整)
    fs::path projectRoot = fs::path(__FILE__).parent_path().parent_path(); 
    fs::path inputFilePath = projectRoot / "rmrobot" / inputFileName;
    fs::path outputFilePath = projectRoot / "rmrobot" / "output" / outputFileName;
    
    // 简单的 dt 预读
    ifstream ifs_pre(inputFilePath);
    if(!ifs_pre.is_open()) { cerr << "Error: " << inputFilePath << " not found." << endl; return -1; }
    string line; getline(ifs_pre, line);
    double t0=0, t1=0.005;
    if(getline(ifs_pre, line)) t0=stod(parseCSVLine(line)[0]);
    if(getline(ifs_pre, line)) t1=stod(parseCSVLine(line)[0]);
    double nominalDt = (t1-t0 > 0) ? (t1-t0) : 0.005;
    ifs_pre.close();

    // ---------------------------------------------------------
    // 初始化
    // ---------------------------------------------------------
    RM65 rm65Robot;
    auto observer = getObserver(&rm65Robot, 5, nominalDt, "real");
    DynamicThresholdDetector thresholdDetector(&rm65Robot); // 这里会自动加载你改好的 .h 里的 params

    int dof = rm65Robot.jointNo();
    Vector q(dof), qd_raw(dof), qd_smooth(dof), qdd_est(dof);
    Vector tau_meas_raw(dof), tau_meas_smooth(dof);
    Vector tau_ext_est(dof), current_thresholds(dof), tau_sim_truth(dof);
    vector<bool> collision_flags(dof);
    
    Vector zero_vec = Vector::Zero(dof);
    Vector prev_qd_smooth = Vector::Zero(dof);

    // === 新增：手动阈值偏置 (Manual Safety Margin) ===
    // 这里的值单位是 Nm，你可以针对每个关节单独调整
    // 经验法则：基座关节(1-3)通常噪音大，给大点；末端关节(4-6)给小点
    Vector manual_offsets(dof);
    manual_offsets << 0.0,  // Joint 1: 加大 3.0 Nm
                      0.0,  // Joint 2: 加大 3.0 Nm
                      0.0,  // Joint 3: 加大 2.5 Nm
                      0.0,  // Joint 4: 加大 1.0 Nm
                      0.0,  // Joint 5: 加大 1.0 Nm
                      0.0;  // Joint 6: 加大 1.0 Nm

    // 如果你想按比例放大（例如整体放大 1.2 倍），也可以定义一个系数
    double safety_scale = 1.1;


    vector<LowPass1st> vel_filters, meas_filters;
    for(int i=0; i<dof; i++) {
        vel_filters.emplace_back(velFc);
        meas_filters.emplace_back(measFc);
    }

    // 绘图数据
    vector<vector<tuple<double, double>>> plot_tau_est(dof), plot_thresh(dof), plot_truth(dof);

    ifstream ifs(inputFilePath);
    getline(ifs, line); // header
    ofstream ofs(outputFilePath);
    ofs << "time";
    for(int i=0; i<dof; i++) ofs << ",est_" << i;
    for(int i=0; i<dof; i++) ofs << ",thr_" << i;
    for(int i=0; i<dof; i++) ofs << ",truth_" << i;
    ofs << endl;

    double prev_time = -1.0;
    bool is_first_step = true;

    cout << "Processing... dt=" << nominalDt << ", velFc=" << velFc << endl;

    while(getline(ifs, line)) {
        vector<string> cols = parseCSVLine(line);
        if(cols.size() < 25) continue;
        double t = stod(cols[0]);
        if(t < startTime) continue;
        if(t > endTime) break;

        for(int i=0; i<dof; i++) {
            q(i) = stod(cols[1+i]);
            qd_raw(i) = stod(cols[7+i]);
            tau_meas_raw(i) = stod(cols[19+i]);
        }

        // 模拟外力
        tau_sim_truth.setZero();
        if(addSimForce && t >= simStartTime) {
            for(int i=0; i<dof; i++) {
                double amp = (i<2)? 3.0 : (i==2? 2.0 : 1.0);
                double val = simAmp * amp * sin(2*M_PI*simFreq*(t - simStartTime));
                tau_sim_truth(i) = val;
                tau_meas_raw(i) -= val; // 扣除真值，测试能否还原
            }
        }

        double dt = nominalDt;
        if(!is_first_step) {
            dt = t - prev_time;
            if(dt <= 1e-5 || dt > 0.1) dt = nominalDt;
        }

        // === 核心信号处理 ===
        if(is_first_step) {
            qd_smooth = qd_raw;
            tau_meas_smooth = tau_meas_raw;
            qdd_est.setZero();
            for(int i=0; i<dof; i++) {
                vel_filters[i].reset(qd_raw(i));
                meas_filters[i].reset(tau_meas_raw(i));
            }
        } else {
            for(int i=0; i<dof; i++) {
                qd_smooth(i) = vel_filters[i].update(qd_raw(i), dt);
                tau_meas_smooth(i) = meas_filters[i].update(tau_meas_raw(i), dt);
            }
            // 计算加速度 (与 Data_Gen.cpp 保持一致)
            qdd_est = (qd_smooth - prev_qd_smooth) / dt;
        }
        prev_qd_smooth = qd_smooth;

        // 1. 观测器
        tau_ext_est = observer->getExternalTorque(q, qd_smooth, tau_meas_smooth, dt);

        // 2. 动力学项计算
        Vector tau_m = rm65Robot.rnea(q, zero_vec, qdd_est, 0.0); // 惯性项 (关键!)
        Vector tau_c = rm65Robot.rnea(q, qd_smooth, zero_vec, 0.0);
        Vector tau_g = rm65Robot.rnea(q, zero_vec, zero_vec, 9.81);
        Vector tau_f = rm65Robot.getFriction(qd_smooth);

        // 3. 动态阈值 (现在传入 4 个参数)
        current_thresholds = thresholdDetector.compute_thresholds(tau_m, tau_c, tau_g, tau_f);

        // === 新增：应用手动调整 ===
        for(int i = 0; i < dof; i++) {
            // 方式 A: 叠加常数 (推荐，适合消除底噪)
            current_thresholds(i) = current_thresholds(i) + manual_offsets(i);
            
            // 方式 B: 比例放大 (可选，适合应对高速动态误差)
            // current_thresholds(i) = current_thresholds(i) * 1.2; 
        }

        // 4. 碰撞检测
        thresholdDetector.check_collision(tau_ext_est, current_thresholds, collision_flags);

        // 保存与绘图
        ofs << t;
        for(int i=0; i<dof; i++) ofs << "," << tau_ext_est(i);
        for(int i=0; i<dof; i++) ofs << "," << current_thresholds(i);
        for(int i=0; i<dof; i++) ofs << "," << tau_sim_truth(i);
        ofs << endl;

        for(int i=0; i<dof; i++) {
            plot_tau_est[i].push_back(make_tuple(t, tau_ext_est(i)));
            plot_thresh[i].push_back(make_tuple(t, current_thresholds(i)));
            if(addSimForce) plot_truth[i].push_back(make_tuple(t, tau_sim_truth(i)));
        }

        prev_time = t;
        is_first_step = false;
    }
    ifs.close(); ofs.close();

    // ---------------------------------------------------------
    // 绘图
    // ---------------------------------------------------------
    Gnuplot gp;
    gp << "set term qt size 1600, 900 title 'Final Validation'\n";
    gp << "set multiplot layout 2,3\n";
    gp << "set grid\n";
    
    auto get_lim = [](const vector<tuple<double, double>>& v) {
        double m = 0; for(auto& t:v) m=max(m, abs(get<1>(t))); return m;
    };

    for(int i=0; i<dof; i++) {
        double lim = max(get_lim(plot_tau_est[i]), get_lim(plot_thresh[i]));
        lim = (lim<1.0)? 2.0 : lim * 1.5;

        gp << "set title 'Joint " << i+1 << "'\n";
        gp << "set yrange [" << -lim << ":" << lim << "]\n";
        gp << "plot '-' w l t 'Thresh' lc 'blue' dt 2, "
           << "'-' w l notitle lc 'blue' dt 2, ";
        if(addSimForce) gp << "'-' w l t 'Truth' lc 'black' dt 1, ";
        gp << "'-' w l t 'Est' lc 'red' lw 2\n";

        gp.send1d(plot_thresh[i]); // 上界
        // 构造下界
        vector<tuple<double,double>> lower;
        for(auto& p: plot_thresh[i]) lower.push_back(make_tuple(get<0>(p), -get<1>(p)));
        gp.send1d(lower);
        
        if(addSimForce) gp.send1d(plot_truth[i]);
        gp.send1d(plot_tau_est[i]);
    }
    gp << "unset multiplot\n";
    
    cout << "Done. Press Enter." << endl;
    cin.get();

    return 0;
}