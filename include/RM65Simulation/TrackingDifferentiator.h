#ifndef TRACKING_DIFFERENTIATOR_H
#define TRACKING_DIFFERENTIATOR_H

#include <cmath>
#include <algorithm>

/**
 * @brief 韩京清非线性跟踪微分器 (Discrete Steepest Tracking Differentiator)
 * 用于从含噪信号 v 中提取平滑的 v 和加速度 a
 */
class TrackingDifferentiator {
private:
    double x1; // 跟踪状态 (平滑后的速度)
    double x2; // 微分状态 (加速度)
    double r;  // 速度因子 (决定跟踪快慢，物理意义为最大加速度)
    double h;  // 滤波因子 (通常取采样周期的倍数，h越大滤波效果越好但延迟越大)

    // fhan 最速控制综合函数
    double fhan(double x1, double x2, double r, double h) {
        double d = r * h * h;
        double a0 = h * x2;
        double y = x1 + a0;
        double a1 = sqrt(d * (d + 8.0 * std::abs(y)));
        double a2 = a0 + copysign(1.0, y) * (a1 - d) / 2.0;
        double sy = (copysign(1.0, y + d) - copysign(1.0, y - d)) / 2.0;
        double a = (a0 + y - a2) * sy + a2;
        double sa = (copysign(1.0, a + d) - copysign(1.0, a - d)) / 2.0;
        return -r * ((a / d) - copysign(1.0, a)) * sa - r * copysign(1.0, a);
    }

public:
    /**
     * @param r_factor 快速因子，越大跟踪越快（但也越敏感）。建议 100~10000
     * @param h_factor 滤波步长，建议设为 3*dt 到 5*dt
     */
    TrackingDifferentiator(double r_factor = 1000.0, double h_factor = 0.02) 
        : x1(0), x2(0), r(r_factor), h(h_factor) {}

    void reset(double init_value) {
        x1 = init_value;
        x2 = 0.0;
    }

    // 核心更新函数：输入含噪速度 v，dt，返回平滑速度和加速度
    void update(double v_input, double dt) {
        // 离散欧拉积分
        // x1(k+1) = x1(k) + h*x2(k)
        // x2(k+1) = x2(k) + h*fhan(...)
        
        // 注意：这里的积分步长用实际 dt
        double fh = fhan(x1 - v_input, x2, r, h);
        x1 += x2 * dt;
        x2 += fh * dt;
    }

    double getSmoothedVelocity() const { return x1; }
    double getAcceleration() const { return x2; }
};

#endif