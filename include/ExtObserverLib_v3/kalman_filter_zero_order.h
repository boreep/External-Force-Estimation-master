#ifndef KALMAN_FILTER_ZERO_ORDER_H
#define KALMAN_FILTER_ZERO_ORDER_H

#include <eigen3/Eigen/Geometry>

class KalmanFilterZeroOrder {
 public:
  /**
   * @brief 构造函数
   * @param a 状态矩阵
   * @param b 控制输入矩阵
   * @param c 观测矩阵
   * @param q 过程噪声协方差
   * @param r 观测噪声协方差
   */
  KalmanFilterZeroOrder(Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c, Eigen::MatrixXd& q, Eigen::MatrixXd& r) 
      : Ad(a), Bd(b), Cd(c), Rd(Eigen::MatrixXd(1, 1)), Qd(Eigen::MatrixXd(1, 1)), Rupd(Eigen::MatrixXd(1, 1)) {
    na = a.rows();
    nb = b.cols();
    nc = c.rows();
    P.resize(na, na);
    X.resize(na);
    Qd = q;
    R = r;
    Rupd = r;
    Rd = r;
    K.resize(na, nc);
    Y.resize(nc, nc);
    I = Eigen::MatrixXd::Identity(na, na);

    AB.resize(na + nb, na + nb);
    AB.setZero();
    AB.block(0, 0, na, na) = a;
    AB.block(0, na, na, nb) = b;
    ABd.resize(na + nb, na + nb);

    AQ.resize(na + na, na + na);
    AQ.block(0, 0, na, na) = a;
    AQ.block(0, na, na, na) = Qd;
    AQ.block(na, na, na, na) = -a.transpose();
    AQd.resize(na + na, na + na);

    // === [NEW] 初始化自适应阈值 ===
    adaptive_threshold = 5.0; // 默认值
  }

  void reset(Eigen::VectorXd& x0) {
    X = x0;
    P = Eigen::MatrixXd::Zero(na, na);
  }

  void updateR(Eigen::MatrixXd& m) {
    Rupd = m * R * m.transpose();
  }

  // === [NEW] 设置自适应阈值接口 ===
  void setAdaptiveThreshold(double val) {
    adaptive_threshold = val;
  }

  Eigen::VectorXd step(Eigen::VectorXd& u, Eigen::VectorXd& y, double dt);

 private:
  Eigen::MatrixXd Ad, Bd, Cd;
  Eigen::MatrixXd R, Rd, Qd, Rupd;
  Eigen::MatrixXd AB, AQ, ABd, AQd;
  Eigen::MatrixXd P, K, Y, I;
  Eigen::VectorXd X;
  int na, nb, nc;

  // === [NEW] 自适应阈值变量 ===
  double adaptive_threshold;

  Eigen::MatrixXd exponential(Eigen::MatrixXd& m, double dt);
  void makeDiscrete(double dt);
};

#endif