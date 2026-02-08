#include <boost/program_options.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>

// 引入项目头文件
#include "RM65Simulation/RM65SimulationModel.h"
#include "RM65Simulation/observers.h" 
#include "gnuplot-iostream.h"

using namespace std;
namespace fs = std::filesystem;

// ============================================================
// 新增：一阶低通滤波器类
// Formula: y = a*x + (1-a)*y_prev
// ============================================================
class LowPassFilter {
private:
    double y_prev;
    double fc;      // 截止频率 (Hz)
    bool is_init;

public:
    LowPassFilter() : y_prev(0.0), fc(10.0), is_init(false) {}

    void init(double cutoff_freq_hz, double initial_value) {
        fc = cutoff_freq_hz;
        y_prev = initial_value;
        is_init = true;
    }

    // 实时更新函数
    double update(double x, double dt) {
        if (!is_init) {
            y_prev = x;
            is_init = true;
            return x;
        }
        // 防止 dt 为 0
        if (dt <= 1e-6) return y_prev;

        double rc = 1.0 / (2.0 * M_PI * fc);
        double alpha = dt / (rc + dt);
        
        // 限制 alpha 范围 (0~1)
        if (alpha > 1.0) alpha = 1.0;
        if (alpha < 0.0) alpha = 0.0;

        double y = alpha * x + (1.0 - alpha) * y_prev;
        y_prev = y;
        return y;
    }
    
    void setCutoff(double freq) { fc = freq; }
};

// CSV 行解析辅助函数
vector<string> parseCSVLine(const string &line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) {
        result.push_back(item);
    }
    return result;
}

int main(int argc, char **argv) {
  // 1. 参数设置
  boost::program_options::options_description description("Allowed options");
  description.add_options()
      ("help,h", "Help message")
      ("parameterType,p", boost::program_options::value<string>()->default_value("real"), "parameter type: normal, small or big.")
      ("inputFile,i", boost::program_options::value<string>()->default_value("joint_data_movej_1_formatted.csv"), "input csv file name")
      ("outputFile,o", boost::program_options::value<string>()->default_value("estimated_force_result.csv"), "output result file name")
      ("observerType,t", boost::program_options::value<int>()->default_value(0), "0:Momentum, 1:Nonlinear, 2:SlidingMode, 3:FilteredDyn, 4/5:Kalman")
      ("startTime,s", boost::program_options::value<double>()->default_value(2.0), "Start time to process (seconds)")
      ("endTime,e", boost::program_options::value<double>()->default_value(5.0), "End time to process (seconds)")
      // --- 模拟外力参数 ---
      ("addSimForce,f", boost::program_options::value<bool>()->default_value(true), "Add simulated sinusoidal external force?")
      ("simAmp,a", boost::program_options::value<double>()->default_value(5.0), "Amplitude of simulated force (Nm)")
      ("simFreq,w", boost::program_options::value<double>()->default_value(0.25), "Frequency of simulated force (Hz)")
      ("simStart,S", boost::program_options::value<double>()->default_value(5.0), "Time to START applying simulated force (seconds)")
      // --- 新增：滤波器参数 ---
      ("velFc,v", boost::program_options::value<double>()->default_value(10.0), "Velocity Low Pass Filter Cutoff Frequency (Hz)");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    cout << description << endl;
    return 0;
  }

  string inputFileName = vm["inputFile"].as<string>();
  string outputFileName = vm["outputFile"].as<string>();
  string parameterType = vm["parameterType"].as<string>();
  int observerType = vm["observerType"].as<int>();
  double startTime = vm["startTime"].as<double>();
  double endTime = vm["endTime"].as<double>();
  double velFc = vm["velFc"].as<double>(); // 获取截止频率
  
  bool addSimForce = vm["addSimForce"].as<bool>();
  double simAmp = vm["simAmp"].as<double>();
  double simFreq = vm["simFreq"].as<double>();
  double simStartTime = vm["simStart"].as<double>();

  // ---------------------------------------------------------
  // 2. 智能路径构建
  // ---------------------------------------------------------
  fs::path srcPath = fs::path(__FILE__).parent_path();
  fs::path projectRoot = srcPath.parent_path(); 
  fs::path inputDir = projectRoot / "rmrobot";
  fs::path outputDir = inputDir / "output";
  fs::path inputFilePath = inputDir / inputFileName;
  fs::path outputFilePath = outputDir / outputFileName;
  
  cout << "Project Root: " << projectRoot << endl;
  cout << "Input File:   " << inputFilePath << endl;
  cout << "Output File:  " << outputFilePath << endl;
  cout << "Velocity LPF: " << velFc << " Hz" << endl; // 打印当前频率
  
  if(addSimForce) {
      cout << "=== SIMULATION MODE ===" << endl;
      cout << "Injecting Sine Wave: Amp=" << simAmp << "Nm, Freq=" << simFreq << "Hz" << endl;
  }

  if (!fs::exists(outputDir)) {
      fs::create_directories(outputDir);
  }

  // ---------------------------------------------------------
  // 3. 智能预读 dt
  // ---------------------------------------------------------
  ifstream ifs_pre(inputFilePath);
  if (!ifs_pre.is_open()) {
      cerr << "Error: Cannot open input file!" << endl;
      return -1;
  }
  
  string line;
  getline(ifs_pre, line); // 跳过表头
  double t0 = 0, t1 = 0.005; 
  bool dt_detected = false;
  if (getline(ifs_pre, line)) t0 = stod(parseCSVLine(line)[0]);
  if (getline(ifs_pre, line)) {
      t1 = stod(parseCSVLine(line)[0]);
      dt_detected = true;
  }
  ifs_pre.close();

  double nominalDt = dt_detected ? (t1 - t0) : 0.005;
  if(nominalDt <= 0) nominalDt = 0.005;

  // ---------------------------------------------------------
  // 4. 初始化
  // ---------------------------------------------------------
  RM65 rm65Robot; 
  std::shared_ptr<ExternalObserverRnea> observer = getObserver(&rm65Robot, observerType, nominalDt, parameterType);

  const int dof = rm65Robot.jointNo(); 
  Vector q(dof), qd(dof), tau_meas(dof), tau_ext_est(dof);
  Vector tau_sim_truth(dof);
  
  // === 初始化滤波器 ===
  vector<LowPassFilter> vel_filters(dof);
  for(int i=0; i<dof; i++) {
      vel_filters[i].setCutoff(velFc);
      // 注意：初始值会在第一次 update 时自动设置
  }

  vector<vector<tuple<double, double>>> plot_vel(dof);
  vector<vector<tuple<double, double>>> plot_tau_meas(dof);
  vector<vector<tuple<double, double>>> plot_tau_est(dof); 
  vector<vector<tuple<double, double>>> plot_tau_sim_gt(dof); 

  // ---------------------------------------------------------
  // 5. 主循环
  // ---------------------------------------------------------
  ifstream ifs(inputFilePath);
  getline(ifs, line); // 跳过表头

  ofstream ofs(outputFilePath);
  ofs << "time";
  for (int i = 1; i <= dof; ++i) ofs << ",est_tau" << i;
  for (int i = 1; i <= dof; ++i) ofs << ",meas_tau" << i;
  for (int i = 1; i <= dof; ++i) ofs << ",gt_tau" << i;
  ofs << endl;

  double prev_time = -1.0;
  bool is_first_step = true;
  int valid_lines_count = 0;

  while (getline(ifs, line)) {
      vector<string> cols = parseCSVLine(line);
      if (cols.size() < 25) continue; 

      double curr_time = stod(cols[0]);

      if (curr_time < startTime) continue; 
      if (curr_time > endTime) break;      

      // 读取数据 (Raw Data)
      for (int i = 0; i < dof; ++i) {
          q(i)        = stod(cols[1 + i]);  
          qd(i)       = stod(cols[7 + i]);   // 这里读到的是原始速度
          tau_meas(i) = stod(cols[19 + i]); 
      }

      // --- 模拟外力控制 ---
      tau_sim_truth.setZero();
      if (addSimForce && curr_time >= simStartTime) {
            for (int i = 0; i < dof; ++i) {
                double ampMultiplier = 1.0;
                if (i < 2) ampMultiplier = 3.0;
                else if (i == 2) ampMultiplier = 2.0;
                
                double currentJointAmp = simAmp * ampMultiplier;
                double sim_val = currentJointAmp * sin(2.0 * M_PI * simFreq * (curr_time - simStartTime));
                
                tau_sim_truth(i) = sim_val;
                tau_meas(i) -= sim_val; 
            }
      }

      // 计算 dt
      double dt = nominalDt; 
      if (!is_first_step) {
          dt = curr_time - prev_time;
          if (dt <= 0 || dt > 0.1) dt = nominalDt; 
      }

      // === 核心修改：对速度进行实时滤波 ===
      // 在计算外力之前，先让速度过一遍低通滤波器
      for(int i = 0; i < dof; i++) {
          qd(i) = vel_filters[i].update(qd(i), dt);
      }
      // =================================

      // 使用滤波后的 qd 进行观测
      tau_ext_est = observer->getExternalTorque(q, qd, tau_meas, dt);

      // Save to file
      ofs << curr_time;
      for (int i = 0; i < dof; ++i) ofs << "," << tau_ext_est(i);
      for (int i = 0; i < dof; ++i) ofs << "," << tau_meas(i);
      for (int i = 0; i < dof; ++i) ofs << "," << tau_sim_truth(i);
      ofs << endl;

      // Save for plotting
      for (int i = 0; i < dof; ++i) {
          plot_vel[i].push_back(make_tuple(curr_time, qd(i))); // 这里画的也是滤波后的速度
          plot_tau_meas[i].push_back(make_tuple(curr_time, tau_meas(i)));
          plot_tau_est[i].push_back(make_tuple(curr_time, tau_ext_est(i)));
          
          if (addSimForce) {
             plot_tau_sim_gt[i].push_back(make_tuple(curr_time, tau_sim_truth(i)));
          }
      }

      prev_time = curr_time;
      is_first_step = false;
      valid_lines_count++;
  }
  
  ifs.close();
  ofs.close();
  
  if (valid_lines_count == 0) {
      cerr << "[Error] No valid data points found." << endl;
      return -1;
  }

  // ---------------------------------------------------------
  // 6. Gnuplot 可视化
  // ---------------------------------------------------------
  Gnuplot gp;
  gp << "set term qt size 1800, 1000 title 'RM65 External Force Observation'\n";
  gp << "set multiplot layout 2,3 title 'Sim Force Starts at " << simStartTime << "s'\n";
  gp << "set grid\n";
  gp << "set xrange [" << startTime << ":" << endTime << "]\n";

  auto get_max_abs = [](const vector<tuple<double, double>>& data) -> double {
      double max_val = 0.0;
      for (const auto& item : data) {
          double val = std::abs(get<1>(item));
          if (val > max_val) max_val = val;
      }
      return max_val;
  };

  for (int i = 0; i < dof; ++i) {
      double max_tau = get_max_abs(plot_tau_meas[i]);
      max_tau = std::max(max_tau, get_max_abs(plot_tau_est[i]));
      if (addSimForce) {
          max_tau = std::max(max_tau, get_max_abs(plot_tau_sim_gt[i]));
      }
      
      double max_vel = get_max_abs(plot_vel[i]);

      max_tau = (max_tau == 0 ? 1.0 : max_tau * 1.2);
      max_vel = (max_vel == 0 ? 1.0 : max_vel * 1.2);

      gp << "set title 'Joint " << (i + 1) << "'\n";
      gp << "set xlabel 'Time (s)'\n";
      gp << "set ylabel 'Torque (Nm)'\n";
      
      gp << "set yrange [" << -max_tau << ":" << max_tau << "]\n";
      gp << "set y2range [" << -max_vel << ":" << max_vel << "]\n";
      
      gp << "set y2tics\n";
      
      gp << "plot ";
      gp << "'-' with lines title 'Measured Tau' lc rgb '#87CEEB' lw 1 axis x1y1, ";
      if (addSimForce) {
          gp << "'-' with lines title 'Sim Ground Truth' lc rgb 'black' dt 2 lw 2 axis x1y1, ";
      }
      gp << "'-' with lines title 'Filtered Vel' lc rgb 'green' dt 3 lw 1 axis x1y2, "; // 标题改为 Filtered Vel
      gp << "'-' with lines title 'Est. Ext Force' lc rgb 'red' lw 2 axis x1y1\n";
         
      gp.send1d(plot_tau_meas[i]);
      if (addSimForce) {
          gp.send1d(plot_tau_sim_gt[i]);
      }
      gp.send1d(plot_vel[i]);      
      gp.send1d(plot_tau_est[i]);
  }
  
  gp << "unset multiplot\n";
  
  cout << "Press Enter to exit..." << endl;
  cin.get();

  return 0;
}