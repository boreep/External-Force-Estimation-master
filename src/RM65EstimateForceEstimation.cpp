#include <boost/program_options.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

// 替换为 RM65 的头文件
#include "RM65Simulation/RM65SimulationModel.h"
#include "RM65Simulation/observers.h" 
#include "gnuplot-iostream.h"

using namespace std;

// 高斯噪声设置
#define MEAN 0.0     // 均值
#define STDDEV 0.01  // 标准差

Vector getConstTauExt() {
  Vector tmp;
  tmp.resize(6);
  tmp << 5, 5, 5, 2, 2, 2;
  return tmp;
}

Vector getVaryTauExt(int i, int startTime, int stopTime) {
  double base1 = sin(M_PI * 2 * (i - startTime) / (stopTime - startTime));
  double base2 = sin(M_PI * 6 * (i - startTime) / (stopTime - startTime));
  Vector tmp;
  tmp.resize(6);
  tmp << 5 * base1 + base2, 5 * base1 + base2, 5 * base1 + base2, 2 * base1 + base2, 2 * base1 + base2, 2 * base1 + base2;
  return tmp;
}

int main(int argc, char **argv) {
  // 设置参数
  boost::program_options::options_description description("Allow options");
  description.add_options()
      ("help,h", "Turn the gain parameter for external observer.")
      ("externalTorqueType,e", boost::program_options::value<string>(), "external torque type: const or vary")
      ("parameterType,p", boost::program_options::value<string>()->default_value("normal"), "parameter type: normal, small or big.")
      ("inputFile,i", boost::program_options::value<string>(), "input file for external observer.")
      ("outputFile,o", boost::program_options::value<string>(), "output file for saving data.")
      ("frequency,f", boost::program_options::value<double>()->default_value(200), "sample Frequency")
      ("observerType,t", boost::program_options::value<int>(),
       "observer type:\n \
    0: momentum observer\n \
    1: nonlinear observer\n \
    2: sliding mode observer\n \
    3: filtered dynamic observer\n \
    4: kalman filter observer(Tayler)\n \
    5: kalman filter observer(Zero order filter)\n");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), vm);
  boost::program_options::notify(vm);
  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 0;
  }
  const string externalTorqueType = vm["externalTorqueType"].as<string>();
  const string parameterType = vm["parameterType"].as<string>();
  
  // 注意：这里仍然读取 Bullet2.83 文件夹下的数据，如果 RM65 需要特定的数据文件请修改路径
  const string inputFilePrefix = std::filesystem::path(__FILE__).parent_path().parent_path().string() + "/rmrobot/";
  string inputFile = inputFilePrefix + vm["inputFile"].as<string>();
  
  const string outputFilePrefix = filesystem::path(__FILE__).parent_path().parent_path().string() + "/rmrobot/output/";
  string outputFile = outputFilePrefix + vm["outputFile"].as<string>();
  const double timeStep = 1 / vm["frequency"].as<double>();
  const int observerType = vm["observerType"].as<int>();

  // ---------------------------------------------------------
  // 修改点：实例化 RM65 机器人模型
  // ---------------------------------------------------------
  RM65 rm65Robot; 
  // 假设 RM65Simulation/observers.h 中也定义了同样的工厂函数 getObserver
  std::shared_ptr<ExternalObserverRnea> observer = getObserver(&rm65Robot, observerType, timeStep, parameterType);

  // 高斯噪声
  default_random_engine generator;
  normal_distribution<double> dist(MEAN, STDDEV);
  
  // 加载数据
  const int dof = rm65Robot.jointNo(); // 获取 RM65 的自由度 (6)
  Vector q(dof), qd(dof), tau(dof), tauExtMeasured(dof), tauExtEstimated(dof);
  q.setZero();
  qd.setZero();
  tau.setZero();
  tauExtMeasured.setZero();
  tauExtEstimated.setZero();
  
  // 画图容器
  vector<vector<tuple<double, double>>> velPlot(dof);
  vector<vector<tuple<double, double>>> tauExtMeasuredPlot(dof), tauExtEstimatedPlot(dof);

  ofstream ofs(outputFile);
  // 这里假设表头依然适用
  ofs << "t, qd1, qd2, qd3, qd4, qd5, qd6,"
      << "tauMea1, tauEst1, tauMea2, tauEst2, tauMea3, tauEst3, tauMea4, tauEst4, tauMea5, tauEst5, tauMea6, tauEst6" << endl;
  
  ifstream ifs(inputFile);
  if (!ifs.is_open()) {
      cerr << "Error: Cannot open input file: " << inputFile << endl;
      return -1;
  }

  string line;
  getline(ifs, line); // 跳过第一行表头
  int row = 0;
  int extStart = 800;
  int extStop = 1200;
  
  while (getline(ifs, line)) {
    int cnt = 0;
    stringstream ss(line);
    string word;
    while (getline(ss, word, ',')) {
      double tmp = atof(word.c_str());
      if (cnt < 6) {
        q(cnt % dof) = tmp;
      } else if (cnt < 12) {
        // 给速度添加噪声
        qd(cnt % dof) = tmp + dist(generator);
      } else if (cnt >= 24 && cnt < 30) {
        // 读取关节力矩 (UR5 数据集通常在 24-30 列是力矩，需确认你的 RM65 数据格式)
        tau(cnt % dof) = tmp;
      }
      ++cnt;
    }
    
    tauExtMeasured.setZero();
    // 在[extStart, extStop]时间内人为添加外力用于测试观测效果
    if (row > extStart && row < extStop) {
      if (externalTorqueType == "const") {
        tauExtMeasured = getConstTauExt();
      } else {
        tauExtMeasured = getVaryTauExt(row, extStart, extStop);
      }
      // 将人为添加的外力从输入的总力矩中减去，模拟“测量力矩包含了外力”的情况
      // 注意：这里的逻辑是用于仿真验证。如果是真实机器人，tau 是直接测量的，不需要手动减。
      for (unsigned int index = 0; index < tau.size(); ++index) {
        tau(index) -= tauExtMeasured(index);
      }
    }
    
    double t = row * timeStep;
    ++row;
    
    // 调用观测器估计外力
    // 这里的 q, qd, tau 传入观测器。由于 RM65SimulationModel 包含了摩擦力计算，
    // 观测器内部会自动执行 tau - Friction - Dynamics，剩下的就是估计的外力。
    tauExtEstimated = observer->getExternalTorque(q, qd, tau, timeStep);
    
    // 保存数据
    ofs << t;
    for (unsigned int i = 0; i < dof; ++i) {
      ofs << "," << qd(i);
    }
    for (unsigned int i = 0; i < dof; ++i) {
      ofs << "," << tauExtMeasured(i) << "," << tauExtEstimated(i);
    }
    ofs << endl;
    
    // 收集画图数据
    for (unsigned int index = 0; index < dof; ++index) {
      velPlot[index].push_back(make_tuple(t, qd(index)));
      tauExtEstimatedPlot[index].push_back(make_tuple(t, tauExtEstimated(index)));
      tauExtMeasuredPlot[index].push_back(make_tuple(t, tauExtMeasured(index)));
    }
  }
  ifs.close();
  ofs.close();

  // 调用gnuplot画图
  Gnuplot gp;
  gp << "set term qt 1 font \",20\" size 1920, 1080\n";
  gp << "set multiplot layout 3,2\n";
  for (unsigned int index = 0; index < dof; ++index) {
    gp << "set ytics nomirror\nset y2tics\n";
    gp << "set xlabel \"time\"\nset ylabel \"torque\"\nset y2label \"velocity\"\n";
    gp << "plot '-' with linespoints linecolor " << to_string(index + 1) << " linewidth 0.6 pointtype " << to_string(index + 1) << " pointsize 1.0 title \"measured_" << to_string(index + 1) << "\" axis x1y1,"
       << "'-' with linespoints linecolor " << to_string(index + 2) << " linewidth 0.6 pointtype " << to_string(index + 2) << " pointsize 1.0 title \"estimated_" << to_string(index + 1) << "\" axis x1y1,"
       << "'-' with linespoints linecolor " << to_string(index + 3) << " linewidth 0.6 pointtype " << to_string(index + 3) << " pointsize 1.0 title \"vel_" << to_string(index + 1) << "\" axis x1y2\n";
    gp.send1d(tauExtMeasuredPlot[index]);
    gp.send1d(tauExtEstimatedPlot[index]);
    gp.send1d(velPlot[index]);
  }
  gp << "unset multiplot\n";
  return 0;
}