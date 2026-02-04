//
// File: RM65_rena_types.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 03-Feb-2026 17:43:16
//

#ifndef RM65_RENA_TYPES_H
#define RM65_RENA_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
namespace RM65Lib {
struct structsol_T {
  double pi_b[42];
  double pi_fr[18];
  double pi_s[66];
};

struct structbaseQR_T {
  int numberOfBaseParameters;
  double permutationMatrix[4356];
  bool motorDynamicsIncluded;
  double beta[1008];
};

} // namespace RM65Lib

#endif
//
// File trailer for RM65_rena_types.h
//
// [EOF]
//
