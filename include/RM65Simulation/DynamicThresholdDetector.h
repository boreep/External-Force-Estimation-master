#ifndef DYNAMIC_THRESHOLD_DETECTOR_H
#define DYNAMIC_THRESHOLD_DETECTOR_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "RM65Simulation/RM65SimulationModel.h"

class DynamicThresholdDetector {
private:
    RM65* robot_ptr;
    int dof;

    // 辨识参数
    // 顺序: {zM, zC, zG, zF_plus, zF_minus, K}
    // 注意：这里只是示例值，你需要运行 threshold_id.py 后把打印出来的结果粘贴到这里！
    double params[6][6] = {
        {0.7088, 0.4299, 0.0876, 0.4402, 0.3811, 0.0500}, // J1
        {0.2413, 0.3210, 0.2005, 0.2222, 0.1884, 0.0500}, // J2
        {0.8260, 0.7019, 0.0000, 0.7412, 1.2807, 0.0500}, // J3
        {0.0328, 1.0883, 0.8178, 0.1312, 0.1688, 0.0500}, // J4
        {0.7829, 0.6300, 0.7542, 0.1451, 0.1256, 0.0500}, // J5
        {0.9386, 10.0000, 4.3481, 0.0390, 0.0958, 0.0500} // J6
    };
public:
    explicit DynamicThresholdDetector(RM65* robot) : robot_ptr(robot) {
        dof = robot_ptr->jointNo();
    }

    /**
     * @brief 计算当前时刻的动态阈值 (Full Dynamics)
     *
     * T = zM*|tau_m| + zC*|tau_c| + zG*|tau_g| + zF+*max(tau_f,0) + zF-*max(-tau_f,0) + K
     *
     * @param tau_m 惯性项力矩 (M*qdd)
     * @param tau_c 科氏/离心项
     * @param tau_g 重力项
     * @param tau_f 摩擦项
     */
    Eigen::VectorXd compute_thresholds(
        const Eigen::VectorXd& tau_m,  // 新增
        const Eigen::VectorXd& tau_c,
        const Eigen::VectorXd& tau_g,
        const Eigen::VectorXd& tau_f
    ) const {
        Eigen::VectorXd thresholds(dof);

        for (int i = 0; i < dof; i++) {
            const double M  = std::abs(tau_m(i));
            const double C  = std::abs(tau_c(i));
            const double G  = std::abs(tau_g(i));
            const double Fp = std::max(tau_f(i), 0.0);
            const double Fm = std::max(-tau_f(i), 0.0);

            // 0:zM, 1:zC, 2:zG, 3:zFp, 4:zFm, 5:K
            thresholds(i) = params[i][0] * M +
                            params[i][1] * C +
                            params[i][2] * G +
                            params[i][3] * Fp +
                            params[i][4] * Fm +
                            params[i][5];
        }
        return thresholds;
    }

    // 碰撞检测逻辑不变
    bool check_collision(
        const Eigen::VectorXd& residual,
        const Eigen::VectorXd& thresholds,
        std::vector<bool>& collisions_out
    ) const {
        bool any_collision = false;
        collisions_out.assign(dof, false);

        for (int i = 0; i < dof; i++) {
            if (std::abs(residual(i)) > thresholds(i)) {
                collisions_out[i] = true;
                any_collision = true;
            }
        }
        return any_collision;
    }
};

#endif // DYNAMIC_THRESHOLD_DETECTOR_H