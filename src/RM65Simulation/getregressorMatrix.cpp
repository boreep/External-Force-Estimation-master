//
// File: getregressorMatrix.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 06-Feb-2026 10:30:29
//

// Include Files
#include "getregressorMatrix.h"
#include "RM65_rena_types.h"
#include "full_regressor_RM65.h"

// Function Definitions
//
// CALCULATE_BASE_REGRESSOR 计算基本惯性参数回归矩阵
//
//  输入:
//    q, qd, q2d : 6x1 double
//    baseQR     : struct (包含 permutationMatrix)
//  输出:
//    phi        : 6x42 double (假设基本参数数量为42)
//
// Arguments    : const double q[6]
//                const double qd[6]
//                const double q2d[6]
//                const struct0_T *baseQR
//                double g
//                double phi[252]
// Return Type  : void
//
namespace RM65Lib {
void getregressorMatrix(const double q[6], const double qd[6],
                        const double q2d[6], const struct0_T *baseQR, double g,
                        double phi[252])
{
  double dv[396];
  //  1. 计算标准回归矩阵 (6x66)
  //  【注意】请确保 standard_regressor_RM65.m 在路径中
  //  2. 获取置换矩阵的前 N_base 列
  //  Coder 需要明确矩阵尺寸，这里我们使用 baseQR 中的数据
  //  假设 baseQR.numberOfBaseParameters 是 42
  //  在 C++ 运行时，你需要确保传入的 permutationMatrix 是正确的
  //  或者使用 baseQR.numberOfBaseParameters 如果它是编译时常量
  //  提取 P 矩阵 (66x42)
  //  3. 计算基本回归矩阵 (6x42)
  full_regressor_RM65(q, qd, q2d, g, dv);
  for (int i{0}; i < 6; i++) {
    for (int i1{0}; i1 < 42; i1++) {
      double d;
      d = 0.0;
      for (int i2{0}; i2 < 66; i2++) {
        d += dv[i + 6 * i2] * baseQR->permutationMatrix[i2 + 66 * i1];
      }
      phi[i + 6 * i1] = d;
    }
  }
}

} // namespace RM65Lib

//
// File trailer for getregressorMatrix.cpp
//
// [EOF]
//
