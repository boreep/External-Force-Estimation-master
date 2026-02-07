//
// File: getFrictionTorque.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 06-Feb-2026 10:30:29
//

// Include Files
#include "getFrictionTorque.h"
#include <cmath>

// Function Definitions
//
// CALCULATE_FRICTION_TORQUE 计算摩擦力矩
//
//  输入:
//    qd    : 6x1 double (关节速度)
//    pi_fr : 18x1 double (摩擦参数: [Fv1, Fc1, Off1, Fv2...])
//  输出:
//    tau_f : 6x1 double (摩擦力矩)
//
// Arguments    : const double qd[6]
//                const double pi_fr[18]
//                double tau_f[6]
// Return Type  : void
//
namespace RM65Lib {
void getFrictionTorque(const double qd[6], const double pi_fr[18],
                       double tau_f[6])
{
  //  假设每关节 3 个参数 (粘性 Fv, 库伦 Fc, 偏置 Off)
  //  对应 frictionRegressor.m 的逻辑: [qd, sign(qd), 1]
  for (int i{0}; i < 6; i++) {
    double d;
    int idx_start;
    //  参数索引 (假设 pi_fr 是按关节顺序排列: J1参数, J2参数...)
    idx_start = i * 3;
    //  计算摩擦力模型: Fv*v + Fc*sign(v) + Off
    d = qd[i];
    tau_f[i] =
        (pi_fr[idx_start] * d + pi_fr[idx_start + 1] * std::tanh(d / 0.01)) +
        pi_fr[idx_start + 2];
  }
}

} // namespace RM65Lib

//
// File trailer for getFrictionTorque.cpp
//
// [EOF]
//
