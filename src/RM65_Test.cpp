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

// CSV 行解析辅助函数
vector<string> parseCSVLine(const string &line) {
    vector<string> result;
    stringstream ss(line);
    string item;
    while (getline(ss, item, ',')) result.push_back(item);
    return result;
}

// ============================================================
// 一阶低通滤波器（速度用）
// y = (1-a)*y + a*x,  a = dt/(tau+dt), tau = 1/(2*pi*fc)
// ============================================================
class LowPass1st {
public:
    explicit LowPass1st(double fc_hz = 10.0) : fc(fc_hz), y(0.0), initialized(false) {}

    void reset(double x0) {
        y = x0;
        initialized = true;
    }

    double update(double x, double dt) {
        if (!initialized) { reset(x); return x; }
        const double tau = 1.0 / (2.0 * M_PI * std::max(1e-6, fc));
        const double a = dt / (tau + dt);
        y = (1.0 - a) * y + a * x;
        return y;
    }

private:
    double fc;
    double y;
    bool initialized;
};

int main(int argc, char **argv) {

  // ---------------------------------------------------------
  // 程序参数
  // ---------------------------------------------------------
  boost::program_options::options_description description("Allowed options");
  description.add_options()
      ("help,h", "Help message")
      ("parameterType,p", boost::program_options::value<string>()->default_value("normal"), "parameter type: normal, small or big.")
      ("inputFile,i", boost::program_options::value<string>()->default_value("Traj3_out_formatted.csv"), "input csv file name")
      ("outputFile,o", boost::program_options::value<string>()->default_value("estimated_force_result.csv"), "output result file name")
      ("observerType,t", boost::program_options::value<int>()->default_value(0), "0:Momentum, 1:Nonlinear, 2:SlidingMode, 3:FilteredDyn, 4/5:Kalman")
      ("startTime,s", boost::program_options::value<double>()->default_value(2.0), "Start time to process (seconds)")
      ("endTime,e", boost::program_options::value<double>()->default_value(30.0), "End time to process (seconds)")
      // --- 模拟外力参数 ---
      ("addSimForce,f", boost::program_options::value<bool>()->default_value(true), "Add simulated sinusoidal external force?")
      ("simAmp,a", boost::program_options::value<double>()->default_value(5.0), "Amplitude of simulated force (Nm)")
      ("simFreq,w", boost::program_options::value<double>()->default_value(0.25), "Frequency of simulated force (Hz)")
      ("simStart,S", boost::program_options::value<double>()->default_value(5.0), "Time to START applying simulated force (seconds)")
      // --- 速度低通参数 ---
      ("velFc", boost::program_options::value<double>()->default_value(10.0), "Velocity low-pass cutoff frequency (Hz), e.g. 8~15");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), vm);
  boost::program_options::notify(vm);

  if (vm.count("help")) {
    cout << description << endl;
    return 0;
  }

  string inputFileName   = vm["inputFile"].as<string>();
  string outputFileName  = vm["outputFile"].as<string>();
  string parameterType   = vm["parameterType"].as<string>();
  int observerType       = vm["observerType"].as<int>();
  double startTime       = vm["startTime"].as<double>();
  double endTime         = vm["endTime"].as<double>();

  bool addSimForce        = vm["addSimForce"].as<bool>();
  double simAmp           = vm["simAmp"].as<double>();
  double simFreq          = vm["simFreq"].as<double>();
  double simStartTime     = vm["simStart"].as<double>();

  double velFc            = vm["velFc"].as<double>();

  // ---------------------------------------------------------
  // 路径与文件初始化
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
  cout << "Vel LPF fc:   " << velFc << " Hz" << endl;

  if(addSimForce) {
      cout << "=== SIMULATION MODE ===" << endl;
      cout << "Injecting Sine Wave: Amp=" << simAmp << "Nm, Freq=" << simFreq << "Hz" << endl;
      cout << "Force Starts at: t=" << simStartTime << "s" << endl;
  }

  if (!fs::exists(outputDir)) fs::create_directories(outputDir);

  // ---------------------------------------------------------
  // 预读 dt
  // ---------------------------------------------------------
  ifstream ifs_pre(inputFilePath);
  if (!ifs_pre.is_open()) { cerr << "Error: Cannot open input file!" << endl; return -1; }
  string line;
  getline(ifs_pre, line);
  double t0 = 0, t1 = 0.005;
  bool dt_detected = false;
  if (getline(ifs_pre, line)) t0 = stod(parseCSVLine(line)[0]);
  if (getline(ifs_pre, line)) { t1 = stod(parseCSVLine(line)[0]); dt_detected = true; }
  ifs_pre.close();
  double nominalDt = dt_detected ? (t1 - t0) : 0.005;
  if(nominalDt <= 0) nominalDt = 0.005;

  cout << "Nominal dt:   " << nominalDt << " s" << endl;

  // ---------------------------------------------------------
  // 对象初始化
  // ---------------------------------------------------------
  RM65 rm65Robot;
  std::shared_ptr<ExternalObserverRnea> observer = getObserver(&rm65Robot, observerType, nominalDt, parameterType);
  DynamicThresholdDetector thresholdDetector(&rm65Robot);

  const int dof = rm65Robot.jointNo();
  Vector q(dof), qd_raw(dof), qd_smooth(dof), tau_meas(dof), tau_ext_est(dof);
  Vector tau_sim_truth(dof);
  Vector current_thresholds(dof);
  vector<bool> collision_flags(dof);

  Vector zero_vec = Vector::Zero(dof);

  // 速度滤波器：每关节一个
  vector<LowPass1st> vel_filters;
  vel_filters.reserve(dof);
  for(int i=0; i<dof; i++) vel_filters.emplace_back(velFc);

  // 绘图容器
  vector<vector<tuple<double, double>>> plot_tau_est(dof);
  vector<vector<tuple<double, double>>> plot_tau_sim_gt(dof);
  vector<vector<tuple<double, double>>> plot_thresh_upper(dof);
  vector<vector<tuple<double, double>>> plot_thresh_lower(dof);

  // 你之前的手动偏置保留（可选）
  Vector threshold_offsets(dof);
  threshold_offsets << 3.0, 3.0, 1.0, 1.0, 1.5, 0.5;

  // ---------------------------------------------------------
  // 主循环
  // ---------------------------------------------------------
  ifstream ifs(inputFilePath);
  if (!ifs.is_open()) { cerr << "Error: Cannot open input file!" << endl; return -1; }

  getline(ifs, line); // 跳过表头
  ofstream ofs(outputFilePath);

    ofs << "time,"
        << "est_tau1,est_tau2,est_tau3,est_tau4,est_tau5,est_tau6,"
        << "thr1,thr2,thr3,thr4,thr5,thr6,"
        << "truth_tau1,truth_tau2,truth_tau3,truth_tau4,truth_tau5,truth_tau6"
        << endl;

  double prev_time = -1.0;
  bool is_first_step = true;
  int valid_lines_count = 0;

  while (getline(ifs, line)) {
      vector<string> cols = parseCSVLine(line);
      if (cols.size() < 25) continue;

      double curr_time = stod(cols[0]);
      if (curr_time < startTime) continue;
      if (curr_time > endTime) break;

      for (int i = 0; i < dof; ++i) {
          q(i)        = stod(cols[1 + i]);
          qd_raw(i)   = stod(cols[7 + i]);   // 原始速度
          tau_meas(i) = stod(cols[19 + i]);
      }

      // 模拟外力注入（与你原来一致）
      tau_sim_truth.setZero();
      if (addSimForce && curr_time >= simStartTime) {
          for (int i = 0; i < dof; ++i) {
              double ampMultiplier = 1.0;
              if (i < 2) ampMultiplier = 3.0;
              else if (i == 2) ampMultiplier = 2.0;

              double currentJointAmp = simAmp * ampMultiplier;
              double sim_val = currentJointAmp * sin(2.0 * M_PI * simFreq * (curr_time - simStartTime));

              tau_sim_truth(i) = sim_val;
              tau_meas(i) -= sim_val;  // 你原来的写法：把外力从测量里“扣掉”
          }
      }

      // dt
      double dt = nominalDt;
      if (!is_first_step) {
          dt = curr_time - prev_time;
          if (dt <= 0 || dt > 0.1) dt = nominalDt;
      }

      // ========================================================
      // 速度低通滤波：qd_raw -> qd_smooth
      // ========================================================
      if (is_first_step) {
          qd_smooth = qd_raw;
          for (int i = 0; i < dof; i++) vel_filters[i].reset(qd_raw(i));
      } else {
          for (int i = 0; i < dof; i++) qd_smooth(i) = vel_filters[i].update(qd_raw(i), dt);
      }

      // --------------------------------------------------------
      // 1) 观测器残差：使用 qd_smooth
      // --------------------------------------------------------
      tau_ext_est = observer->getExternalTorque(q, qd_smooth, tau_meas, dt);

      // --------------------------------------------------------
      // 2) 动态阈值（新方法：不含惯性项 + 摩擦正负拆分）
      //    tau_c / tau_f 用 qd_smooth，tau_g 用 q
      // --------------------------------------------------------
      Vector tau_c = rm65Robot.rnea(q, qd_smooth, zero_vec, 0.0);
      Vector tau_g = rm65Robot.rnea(q, zero_vec,  zero_vec, 9.8);
      Vector tau_f = rm65Robot.getFriction(qd_smooth);

      current_thresholds = thresholdDetector.compute_thresholds(tau_c, tau_g, tau_f);

      // --- 可选：你原来的手动调整保留 ---
      current_thresholds = current_thresholds * 1.2;
      current_thresholds += threshold_offsets;

      // --------------------------------------------------------
      // 3) 碰撞检测
      // --------------------------------------------------------
      thresholdDetector.check_collision(tau_ext_est, current_thresholds, collision_flags);

      // 保存
    ofs << curr_time;
    for (int i = 0; i < dof; ++i) ofs << "," << tau_ext_est(i);
    for (int i = 0; i < dof; ++i) ofs << "," << current_thresholds(i);
    for (int i = 0; i < dof; ++i) ofs << "," << tau_sim_truth(i);   // 新增
    ofs << endl;

      // 绘图数据
      for (int i = 0; i < dof; ++i) {
          plot_tau_est[i].push_back(make_tuple(curr_time, tau_ext_est(i)));
          plot_thresh_upper[i].push_back(make_tuple(curr_time, current_thresholds(i)));
          plot_thresh_lower[i].push_back(make_tuple(curr_time, -current_thresholds(i)));
          if (addSimForce) plot_tau_sim_gt[i].push_back(make_tuple(curr_time, tau_sim_truth(i)));
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
  // Gnuplot 可视化
  // ---------------------------------------------------------
  Gnuplot gp;
  gp << "set term qt size 1800, 1000 title 'RM65 Dynamic Threshold Test (Velocity LPF)'\n";
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
      double max_y = get_max_abs(plot_tau_est[i]);
      max_y = std::max(max_y, get_max_abs(plot_thresh_upper[i]));
      if (addSimForce) max_y = std::max(max_y, get_max_abs(plot_tau_sim_gt[i]));
      max_y = (max_y == 0 ? 1.0 : max_y * 1.5);

      gp << "set title 'Joint " << (i + 1) << "'\n";
      gp << "set xlabel 'Time (s)'\n";
      gp << "set ylabel 'Torque (Nm)'\n";
      gp << "set yrange [" << -max_y << ":" << max_y << "]\n";

      gp << "plot ";
      gp << "'-' with lines title '+Threshold' lc rgb 'blue' dt 2 lw 2, ";
      gp << "'-' with lines title '-Threshold' lc rgb 'blue' dt 2 lw 2, ";
      if (addSimForce) {
          gp << "'-' with lines title 'Truth' lc rgb 'black' dt 4 lw 1, ";
      }
      gp << "'-' with lines title 'Est. Force' lc rgb 'red' lw 2\n";

      gp.send1d(plot_thresh_upper[i]);
      gp.send1d(plot_thresh_lower[i]);
      if (addSimForce) gp.send1d(plot_tau_sim_gt[i]);
      gp.send1d(plot_tau_est[i]);
  }

  gp << "unset multiplot\n";

  cout << "Validation finished." << endl;
  cout << "Press Enter to exit..." << endl;
  cin.get();

  return 0;
}
