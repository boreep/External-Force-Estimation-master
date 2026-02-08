#include "ExtObserverLib_v3/kalman_filter_zero_order.h"

Eigen::MatrixXd KalmanFilterZeroOrder::exponential(Eigen::MatrixXd& m, double dt) {
  // exp(m*dt) = 累加((m*dt)^n/n!)
  Eigen::MatrixXd res = Eigen::MatrixXd::Identity(m.rows(), m.cols());
  Eigen::MatrixXd zhankaixiang = res;

  // 展开项数5
  for (int i = 1; i < 5; i++) {
    zhankaixiang *= (m * dt) / i;
    res += zhankaixiang;
  }

  return res;
}
// ---------------------------------------------------------
// 改进点1: 增加自适应系数 adaptive_factor
// 改进点2: 修复 R 的使用 Bug
// ---------------------------------------------------------

void KalmanFilterZeroOrder::makeDiscrete(double dt) {
  // 1. 系统矩阵离散化 (保持原逻辑，虽然泰勒展开效率低但通用)
  ABd = exponential(AB, dt);
  Ad = ABd.block(0, 0, na, na);
  Bd = ABd.block(0, na, na, nb);

  // 2. [BUG FIX] 测量噪声离散化
  // 原代码: Rd = R / dt;  <-- 错误，使用了并未更新的 R
  // 修正为: 使用 updateR() 更新过的 Rupd
  Rd = Rupd / dt; 

  // 3. 过程噪声离散化
  AQd = exponential(AQ, dt);
  Qd = AQd.block(0, na, na, na) * AQd.block(0, 0, na, na).transpose();
}

Eigen::VectorXd KalmanFilterZeroOrder::step(Eigen::VectorXd& u, Eigen::VectorXd& y, double dt) {
  makeDiscrete(dt);

  // --- 预测步骤 ---
  X = Ad * X + Bd * u;
  P = Ad * P * Ad.transpose() + Qd;

  // --- 创新/残差计算 ---
  // Innovation: y_tilde = y - C * X
  Eigen::VectorXd innovation = y - Cd * X;

  // --- [NEW] 自适应过程噪声 (Adaptive Q) ---
  // 计算归一化残差平方 (Mahalanobis Distance 简化版)
  // 如果残差过大，说明发生碰撞，临时放大 P 阵或 Q 阵，让滤波器“相信”测量值
  double error_sq = innovation.squaredNorm();
  double threshold = 5.0; // 需要根据实际噪声水平调整
  
  if (error_sq > threshold) {
      // 碰撞发生：此时模型预测失效，大幅增加预测协方差
      // 这样卡尔曼增益 K 会变大，权重向“测量值”倾斜，响应更快
      double scale = 1.0 + (error_sq / threshold); // 简单的自适应律
      P = P * scale; 
  }

  // --- 更新步骤 ---
  Y = Cd * P * Cd.transpose() + Rd;
  K = P * Cd.transpose() * Y.inverse(); // 卡尔曼增益

  // 改正
  X += K * innovation;
  P = (I - K * Cd) * P;

  return X;
}