//
// File: RM65_rena.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 06-Feb-2026 10:30:29
//

// Include Files
#include "RM65_rena.h"
#include "RM65_rena_types.h"
#include "full_regressor_RM65.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
namespace RM65Lib {
static int div_nzp_s32(int numerator);

}

// Function Definitions
//
// Arguments    : int numerator
// Return Type  : int
//
namespace RM65Lib {
static int div_nzp_s32(int numerator)
{
  int quotient;
  unsigned int tempAbsQuotient;
  if (numerator < 0) {
    tempAbsQuotient = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    tempAbsQuotient = static_cast<unsigned int>(numerator);
  }
  tempAbsQuotient /= 66U;
  if (numerator < 0) {
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

//
// ---------------------------------------------------------------------
//  该函数用于计算机器人在特定运动状态下的预测力矩
//
//  输入参数:
//    qi       - 关节位置向量 (6x1)
//    qdi      - 滤波后的关节速度向量 (6x1)
//    q2di     - 估算的关节加速度向量 (6x1)
//    baseQR   - 包含机器人结构信息的结构体 (由 base_params_qr 生成)
//    pi_b     - 辨识出的机器人基参数 (Base Parameters)
//    pi_fr    - 辨识出的摩擦力参数 (Friction Parameters)
//
//  输出参数:
//    tau_pred - 预测的关节力矩向量 (6x1, 单位: Nm)
//  ---------------------------------------------------------------------
//
// Arguments    : const double q[6]
//                const double qd[6]
//                const double q2d[6]
//                const struct0_T *baseQR
//                const struct1_T *sol
//                double g
//                double tau_pred[6]
// Return Type  : void
//
void RM65_rena(const double q[6], const double qd[6], const double q2d[6],
               const struct0_T *baseQR, const struct1_T *sol, double g,
               double tau_pred[6])
{
  double b_Ybi_data[504];
  double Ybi_data[396];
  double Yi[396];
  double Yfrctni[108];
  double b_sol[60];
  double bkj;
  int b_i;
  int coffset;
  int nc;
  //  包含电机反射惯量的模型 (6x66 矩阵)
  full_regressor_RM65(q, qd, q2d, g, Yi);
  //  2. 将全参数回归矩阵映射到基参数空间
  //  使用置换矩阵的索引截取前 n 个独立列
  if (baseQR->numberOfBaseParameters < 1) {
    nc = 0;
  } else {
    nc = baseQR->numberOfBaseParameters;
  }
  for (int j{0}; j < nc; j++) {
    int boffset;
    coffset = j * 6;
    boffset = j * 66;
    for (int i{0}; i < 6; i++) {
      Ybi_data[coffset + i] = 0.0;
    }
    for (int k{0}; k < 66; k++) {
      int aoffset;
      aoffset = k * 6;
      b_i = boffset + k;
      bkj = baseQR->permutationMatrix[b_i % 66 + 66 * div_nzp_s32(b_i)];
      for (int i{0}; i < 6; i++) {
        b_i = coffset + i;
        Ybi_data[b_i] += Yi[aoffset + i] * bkj;
      }
    }
  }
  //  3. 计算摩擦力回归矩阵
  //  通常包含库伦摩擦和粘性摩擦项
  //  ----------------------------------------------------------------------
  //  The function computes friction regressor for each joint of the robot.
  //  Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
  //  ---------------------------------------------------------------------
  std::memset(&Yfrctni[0], 0, 108U * sizeof(double));
  for (int i{0}; i < 6; i++) {
    b_i = 3 * (i + 1) - 3;
    bkj = qd[i];
    Yfrctni[i + 6 * b_i] = bkj;
    Yfrctni[i + 6 * (b_i + 1)] = std::tanh(bkj / 0.01);
    Yfrctni[i + 6 * (b_i + 2)] = 1.0;
  }
  //  4. 合成总回归矩阵并乘以辨识出的参数向量
  //  tau = [惯性与重力项  摩擦项] * [基参数; 摩擦参数]
  for (b_i = 0; b_i < nc; b_i++) {
    for (int j{0}; j < 6; j++) {
      coffset = j + 6 * b_i;
      b_Ybi_data[coffset] = Ybi_data[coffset];
    }
  }
  for (b_i = 0; b_i < 18; b_i++) {
    for (int j{0}; j < 6; j++) {
      b_Ybi_data[j + 6 * (b_i + nc)] = Yfrctni[j + 6 * b_i];
    }
  }
  std::copy(&sol->pi_b[0], &sol->pi_b[42], &b_sol[0]);
  std::copy(&sol->pi_fr[0], &sol->pi_fr[18], &b_sol[42]);
  for (b_i = 0; b_i < 6; b_i++) {
    bkj = 0.0;
    for (int j{0}; j < 60; j++) {
      bkj += b_Ybi_data[b_i + 6 * j] * b_sol[j];
    }
    tau_pred[b_i] = bkj;
  }
}

} // namespace RM65Lib

//
// File trailer for RM65_rena.cpp
//
// [EOF]
//
