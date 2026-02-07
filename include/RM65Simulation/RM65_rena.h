//
// File: RM65_rena.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 06-Feb-2026 10:30:29
//

#ifndef RM65_RENA_H
#define RM65_RENA_H

// Include Files
#include "RM65_rena_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace RM65Lib {
extern void RM65_rena(const double q[6], const double qd[6],
                      const double q2d[6], const struct0_T *baseQR,
                      const struct1_T *sol, double g, double tau_pred[6]);

}

#endif
//
// File trailer for RM65_rena.h
//
// [EOF]
//
