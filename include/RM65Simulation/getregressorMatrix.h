//
// File: regressorMatrix.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 03-Feb-2026 17:43:16
//

#ifndef REGRESSORMATRIX_H
#define REGRESSORMATRIX_H

// Include Files
#include "RM65_rena_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace RM65Lib {
extern void getregressorMatrix(const double q[6], const double qd[6],
                            const double q2d[6], const structbaseQR_T *baseQR,
                            double phi[252]);

}

#endif
//
// File trailer for regressorMatrix.h
//
// [EOF]
//
