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

    // Python 辨识输出参数
    // 顺序: {zC, zG, zF_plus, zF_minus, K}
    double params[6][5] = {
        {0.0000, 0.2112, 0.0000, 0.0563, 2.0982}, // Joint 1
        {2.0000, 0.2726, 0.0171, 0.0467, 0.7874}, // Joint 2
        {0.5666, 0.0000, 0.1721, 0.1703, 0.9303}, // Joint 3
        {2.0000, 0.5929, 0.0623, 0.1249, 0.7649}, // Joint 4
        {1.6747, 0.5943, 0.0000, 0.0899, 0.9997}, // Joint 5
        {2.0000, 2.0000, 0.0000, 0.0000, 1.2731} // Joint 6
    };
public:
    explicit DynamicThresholdDetector(RM65* robot) : robot_ptr(robot) {
        dof = robot_ptr->jointNo();
    }

    /**
     * @brief 计算当前时刻的动态阈值（slack + 摩擦正负拆分版）
     *
     * 阈值公式（每关节 i）：
     *   T_i = zC*|tau_c| + zG*|tau_g| + zF+*max(tau_f,0) + zF-*max(-tau_f,0) + K
     *
     * @param tau_c 科氏/离心项（关节力矩）
     * @param tau_g 重力项（关节力矩）
     * @param tau_f 摩擦项（带符号，关节力矩）
     * @return Eigen::VectorXd 阈值向量（dof 维）
     */
    Eigen::VectorXd compute_thresholds(
        const Eigen::VectorXd& tau_c,
        const Eigen::VectorXd& tau_g,
        const Eigen::VectorXd& tau_f
    ) const {
        Eigen::VectorXd thresholds(dof);

        for (int i = 0; i < dof; i++) {
            const double C  = std::abs(tau_c(i));
            const double G  = std::abs(tau_g(i));
            const double Fp = std::max(tau_f(i), 0.0);
            const double Fm = std::max(-tau_f(i), 0.0);

            thresholds(i) = params[i][0] * C +
                            params[i][1] * G +
                            params[i][2] * Fp +
                            params[i][3] * Fm +
                            params[i][4];
        }
        return thresholds;
    }

    /**
     * @brief 检测碰撞：若 |residual(i)| > thresholds(i) 则该关节碰撞
     *
     * @param residual 观测器输出残差（外部力矩估计/残差）
     * @param thresholds 动态阈值
     * @param collisions_out 输出：每关节是否碰撞
     * @return true 任意关节碰撞
     */
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




// #ifndef DYNAMIC_THRESHOLD_DETECTOR_H
// #define DYNAMIC_THRESHOLD_DETECTOR_H

// #include <vector>
// #include <cmath>
// #include <eigen3/Eigen/Dense>
// #include "RM65Simulation/RM65SimulationModel.h"

// class DynamicThresholdDetector {
// private:
//     RM65* robot_ptr;
//     int dof;
    
//     // --- 这里填入 Python 脚本计算出的参数 ---
//     // 顺序: {zeta_coriolis, zeta_gravity, zeta_friction, constant_margin}
//     // 示例值 (请替换为实际辨识结果):
//     // double params[6][4] = {
//     //     {0.0000, 0.2781, 0.0000, 2.3517}, // Joint 1
//     //     {1.0919, 0.2752, 0.1060, 0.1514}, // Joint 2
//     //     {0.1958, 0.0578, 0.2045, 0.8510}, // Joint 3
//     //     {0.5193, 0.5604, 0.2163, 0.7337}, // Joint 4
//     //     {2.0000, 0.5013, 0.0439, 1.8105}, // Joint 5
//     //     {2.0000, 2.0000, 0.0000, 2.1267} // Joint 6
//     // };
//     double params[6][5] = {
//         {1.3129, 1.5000, 0.1611, 0.0000, 2.1119},
//         {0.0000, 0.0000, 0.6597, 0.0000, 0.9517},
//         {0.0000, 0.6902, 0.1378, 0.0000, 1.2597},
//         {0.0000, 1.5000, 0.5388, 0.0000, 1.6065},
//         {0.0000, 0.0000, 0.8957, 0.0000, 1.6834},
//         {1.0623, 1.5000, 0.9476, 0.0000, 1.7445}
//     };
//     Eigen::VectorXd zero_vec;

// public:
//     DynamicThresholdDetector(RM65* robot) : robot_ptr(robot) {
//         dof = robot->jointNo();
//         zero_vec = Eigen::VectorXd::Zero(dof);
//     }

//     /**
//      * @brief 计算当前时刻的动态阈值
//      * @param q 当前关节位置
//      * @param qd 当前关节速度
//      * @return Eigen::VectorXd 包含6个关节的阈值
//      */
//     // Eigen::VectorXd compute_thresholds(Eigen::VectorXd& q, Eigen::VectorXd& qd) {
//     //     Eigen::VectorXd thresholds(dof);
        
//     //     // 1. 计算各项动力学分量 (复用数据采集时的逻辑)
//     //     // 重力项 (g=9.8)
//     //     Eigen::VectorXd tau_g = robot_ptr->rnea(q, zero_vec, zero_vec, 9.8);
        
//     //     // 科氏力项 (g=0, 忽略加速度)
//     //     Eigen::VectorXd tau_c = robot_ptr->rnea(q, qd, zero_vec, 0.0);
        
//     //     // 摩擦力项
//     //     Eigen::VectorXd tau_f = robot_ptr->getFriction(qd);
        
//     //     // 2. 合成阈值
//     //     for(int i=0; i<dof; i++) {
//     //         double dyn_part = params[i][0] * std::abs(tau_c(i)) + 
//     //                           params[i][1] * std::abs(tau_g(i)) + 
//     //                           params[i][2] * std::abs(tau_f(i));
            
//     //         thresholds(i) = dyn_part + params[i][3];
//     //     }
        
//     //     return thresholds;
//     // }
//     // 修改计算函数，接收 tau_m (惯性项)
//     Eigen::VectorXd compute_thresholds_full(
//         const Eigen::VectorXd& tau_m, // 新增参数
//         const Eigen::VectorXd& tau_c, 
//         const Eigen::VectorXd& tau_g, 
//         const Eigen::VectorXd& tau_f) 
//     {
//         Eigen::VectorXd thresholds(dof);
//         for(int i=0; i<dof; i++) {
//             thresholds(i) = params[i][0] * std::abs(tau_m(i)) + 
//                             params[i][1] * std::abs(tau_c(i)) + 
//                             params[i][2] * std::abs(tau_g(i)) + 
//                             params[i][3] * std::abs(tau_f(i)) + 
//                             params[i][4]; // Const
//         }
//         return thresholds;
//     }

//     /**
//      * @brief 检测碰撞
//      * @return bool 如果任意关节发生碰撞返回 true
//      */
//     bool check_collision(const Eigen::VectorXd& residual, const Eigen::VectorXd& thresholds, std::vector<bool>& collisions_out) {
//         bool any_collision = false;
//         collisions_out.resize(dof);
        
//         for(int i=0; i<dof; i++) {
//             if (std::abs(residual(i)) > thresholds(i)) {
//                 collisions_out[i] = true;
//                 any_collision = true;
//             } else {
//                 collisions_out[i] = false;
//             }
//         }
//         return any_collision;
//     }
// };

// #endif