//
// File: getregressorMatrix.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 06-Feb-2026 10:30:29
//

#ifndef GETREGRESSORMATRIX_H
#define GETREGRESSORMATRIX_H

// Include Files
#include "RM65_rena_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace RM65Lib {
extern void getregressorMatrix(const double q[6], const double qd[6],
                               const double q2d[6], const struct0_T *baseQR,
                               double g, double phi[252]);

}

#endif
//
// File trailer for getregressorMatrix.h
//
// [EOF]
//
