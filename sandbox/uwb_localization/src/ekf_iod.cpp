//
// File: ekf_iod.cpp
//
// Code generated for Simulink model 'ekf_iod'.
//
// Model version                  : 1.296
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Sat Oct 24 22:13:33 2015
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. Traceability
// Validation result: Not run
//
#include "ekf_iod.h"
#define NumBitsPerChar                 8U

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern "C" {
  extern real_T rtGetInf(void);
  extern real32_T rtGetInfF(void);
  extern real_T rtGetMinusInf(void);
  extern real32_T rtGetMinusInfF(void);
}                                      // extern "C"
  extern "C"
{
  extern real_T rtGetNaN(void);
  extern real32_T rtGetNaNF(void);
}                                      // extern "C"

extern "C" {
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  extern void rt_InitInfAndNaN(size_t realSize);
  extern boolean_T rtIsInf(real_T value);
  extern boolean_T rtIsInfF(real32_T value);
  extern boolean_T rtIsNaN(real_T value);
  extern boolean_T rtIsNaNF(real32_T value);
  typedef struct {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  } BigEndianIEEEDouble;

  typedef struct {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  } LittleEndianIEEEDouble;

  typedef struct {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  } IEEESingle;
}                                      // extern "C"
  extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   // do nothing
# else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif

extern "C" {
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}
  extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0 } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C" {
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  boolean_T rtIsNaN(real_T value)
  {
    return (boolean_T)((value!=value) ? 1U : 0U);
  }

  // Test if single-precision value is not a number
  boolean_T rtIsNaNF(real32_T value)
  {
    return (boolean_T)(((value!=value) ? 1U : 0U));
  }
}
  real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = (rtNaN);
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

// Model step function
void ekfIodClass::step(real_T arg_dists[4], real_T arg_deltat, real_T arg_imu,
  uint8_T arg_nodeId, boolean_T arg_calibEnable, real_T arg_ancs[12], real_T
  arg_x_est[6], boolean_T *arg_outlierDetected)
{
  static const int8_T b[6] = { 0, 0, 0, 0, 0, 1 };

  static const int8_T c[6] = { 0, 0, 0, 0, 1, 0 };

  static const int8_T d[6] = { 0, 0, 0, 1, 0, 0 };

  real_T t;
  boolean_T rtb_outlierDetected;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  real_T disp_pre_idx_0;
  real_T disp_pre_idx_1;
  real_T disp_pre_idx_2;
  UNUSED_PARAMETER(arg_imu);

  // MATLAB Function: '<S1>/ekf prediction' incorporates:
  //   Inport: '<Root>/deltat'
  //   UnitDelay: '<S1>/Unit Delay1'
  //   UnitDelay: '<S1>/Unit Delay2'

  // MATLAB Function 'ekf_iod/ekf prediction': '<S3>:1'
  //  x_pre: 6 by 1, [pos vel]; P_pre: 6 by 6
  //  x_est: 6 by 1, pos & vel; P_est: 6 by 6, covariance at last update;
  //  deltat: time difference between 2 effective rangings or estimations
  //  parameters acc_xy & acc_z for producing Q
  //  acceleration magnitude of x-y
  //  acceleration magnitude of z
  // '<S3>:1:11' A = [...
  // '<S3>:1:12'     1       0    	0   	deltat  0       0;...
  // '<S3>:1:13'     0       1   	0   	0       deltat  0;...
  // '<S3>:1:14'     0       0       1       0       0       deltat;...
  // '<S3>:1:15'     0       0       0       1       0       0;...
  // '<S3>:1:16'     0       0       0       0       1       0;...
  // '<S3>:1:17'     0       0       0       0       0       1];
  ekf_iod_B.A[0] = 1.0;
  ekf_iod_B.A[6] = 0.0;
  ekf_iod_B.A[12] = 0.0;
  ekf_iod_B.A[18] = arg_deltat;
  ekf_iod_B.A[24] = 0.0;
  ekf_iod_B.A[30] = 0.0;
  ekf_iod_B.A[1] = 0.0;
  ekf_iod_B.A[7] = 1.0;
  ekf_iod_B.A[13] = 0.0;
  ekf_iod_B.A[19] = 0.0;
  ekf_iod_B.A[25] = arg_deltat;
  ekf_iod_B.A[31] = 0.0;
  ekf_iod_B.A[2] = 0.0;
  ekf_iod_B.A[8] = 0.0;
  ekf_iod_B.A[14] = 1.0;
  ekf_iod_B.A[20] = 0.0;
  ekf_iod_B.A[26] = 0.0;
  ekf_iod_B.A[32] = arg_deltat;
  for (i = 0; i < 6; i++) {
    ekf_iod_B.A[3 + 6 * i] = d[i];
  }

  for (i = 0; i < 6; i++) {
    ekf_iod_B.A[4 + 6 * i] = c[i];
  }

  for (i = 0; i < 6; i++) {
    ekf_iod_B.A[5 + 6 * i] = b[i];
  }

  // '<S3>:1:19' Q = [deltat^4/4*acc_xy^2           0                       0               deltat^3/2*acc_xy^2             0                       0;... 
  // '<S3>:1:20'     0                     deltat^4/4*acc_xy^2              0                       0               deltat^3/2*acc_xy^2             0;... 
  // '<S3>:1:21'     0                              0               deltat^4/4*acc_z^2              0                       0               deltat^3/2*acc_z^2;... 
  // '<S3>:1:22'     deltat^3/2*acc_xy^2            0                       0               deltat^2/2*acc_xy^2             0                       0;... 
  // '<S3>:1:23'     0                     deltat^3/2*acc_xy^2              0                       0               deltat^2*acc_xy^2               0;... 
  // '<S3>:1:24'     0                              0               deltat^3/2*acc_z^2              0                       0               deltat^2*acc_z^2]; 
  ekf_iod_B.Q[0] = rt_powd_snf(arg_deltat, 4.0) / 4.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[6] = 0.0;
  ekf_iod_B.Q[12] = 0.0;
  ekf_iod_B.Q[18] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[24] = 0.0;
  ekf_iod_B.Q[30] = 0.0;
  ekf_iod_B.Q[1] = 0.0;
  ekf_iod_B.Q[7] = rt_powd_snf(arg_deltat, 4.0) / 4.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[13] = 0.0;
  ekf_iod_B.Q[19] = 0.0;
  ekf_iod_B.Q[25] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[31] = 0.0;
  ekf_iod_B.Q[2] = 0.0;
  ekf_iod_B.Q[8] = 0.0;
  ekf_iod_B.Q[14] = rt_powd_snf(arg_deltat, 4.0) / 4.0 * (ekf_iod_DW.acc_z *
    ekf_iod_DW.acc_z);
  ekf_iod_B.Q[20] = 0.0;
  ekf_iod_B.Q[26] = 0.0;
  ekf_iod_B.Q[32] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_z *
    ekf_iod_DW.acc_z);
  ekf_iod_B.Q[3] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[9] = 0.0;
  ekf_iod_B.Q[15] = 0.0;
  ekf_iod_B.Q[21] = arg_deltat * arg_deltat / 2.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[27] = 0.0;
  ekf_iod_B.Q[33] = 0.0;
  ekf_iod_B.Q[4] = 0.0;
  ekf_iod_B.Q[10] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[16] = 0.0;
  ekf_iod_B.Q[22] = 0.0;
  ekf_iod_B.Q[28] = arg_deltat * arg_deltat * (ekf_iod_DW.acc_xy *
    ekf_iod_DW.acc_xy);
  ekf_iod_B.Q[34] = 0.0;
  ekf_iod_B.Q[5] = 0.0;
  ekf_iod_B.Q[11] = 0.0;
  ekf_iod_B.Q[17] = rt_powd_snf(arg_deltat, 3.0) / 2.0 * (ekf_iod_DW.acc_z *
    ekf_iod_DW.acc_z);
  ekf_iod_B.Q[23] = 0.0;
  ekf_iod_B.Q[29] = 0.0;
  ekf_iod_B.Q[35] = arg_deltat * arg_deltat * (ekf_iod_DW.acc_z *
    ekf_iod_DW.acc_z);

  // '<S3>:1:26' x_pre = A*x_upd_1;
  // '<S3>:1:27' P_pre = A*P_upd_1*A' + Q;
  for (i = 0; i < 6; i++) {
    for (i_1 = 0; i_1 < 6; i_1++) {
      ekf_iod_B.A_m[i + 6 * i_1] = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        ekf_iod_B.A_m[i + 6 * i_1] += ekf_iod_B.A[6 * i_0 + i] *
          ekf_iod_DW.UnitDelay2_DSTATE[6 * i_1 + i_0];
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (i_1 = 0; i_1 < 6; i_1++) {
      disp_pre_idx_0 = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        disp_pre_idx_0 += ekf_iod_B.A_m[6 * i_0 + i] * ekf_iod_B.A[6 * i_0 + i_1];
      }

      ekf_iod_B.P_pre[i + 6 * i_1] = ekf_iod_B.Q[6 * i_1 + i] + disp_pre_idx_0;
    }
  }

  //  d_hat=norm(p_hat' - ancs(:, nodeId), 2);
  //  H=[(p_hat' - ancs(:, nodeId))/d_hat 0 0 0];
  //  innov = distsCalibed(nodeId) - d_hat;
  //  S = R + H*P_pre*H';
  //
  //  outlierDetected = false;
  //
  //  if innov^2/S > ADMIT_RATIO^2
  //      outlierDetected = true;
  //      vmax = zeros(3, 1);
  //      v_prev = x_prev(4:6);
  //      for i = 1:3
  //          vmax(i) = max(abs(v_prev(i)-sqrt(P_prev(i,i))), abs(v_prev(i)+sqrt(P_prev(i,i)))); 
  //          Q(i,i) = Q(i,i) + diag([vmax(i).^2 * deltat^2; zeros(3,1)]);
  //      end
  //
  //      %Current prediction is the same but covariance is increased to account 
  //      %for uncertainty of position after a longer time
  //      x_pre = x_prev;
  //      P_pre = P_prev + Q;
  //  end
  for (i = 0; i < 6; i++) {
    ekf_iod_B.x_pre[i] = 0.0;
    for (i_1 = 0; i_1 < 6; i_1++) {
      ekf_iod_B.x_pre[i] += ekf_iod_B.A[6 * i_1 + i] *
        ekf_iod_DW.UnitDelay1_DSTATE[i_1];
    }
  }

  // End of MATLAB Function: '<S1>/ekf prediction'

  // MATLAB Function: '<S1>/Range Calibration' incorporates:
  //   Inport: '<Root>/calibEnable'
  //   Inport: '<Root>/dists'
  //   Inport: '<Root>/nodeId'

  // MATLAB Function 'ekf_iod/Range Calibration': '<S2>:1'
  // '<S2>:1:3' distsCalibed = dists;
  ekf_iod_B.distsCalibed[0] = arg_dists[0];
  ekf_iod_B.distsCalibed[1] = arg_dists[1];
  ekf_iod_B.distsCalibed[2] = arg_dists[2];
  ekf_iod_B.distsCalibed[3] = arg_dists[3];

  // '<S2>:1:4' if calibEnable
  if (arg_calibEnable) {
    // '<S2>:1:5' if 0 < dists(nodeId) && dists(nodeId) < 1.5
    if ((0.0 < arg_dists[arg_nodeId - 1]) && (arg_dists[arg_nodeId - 1] < 1.5))
    {
      // '<S2>:1:6' distsCalibed(nodeId) = 1.0447 * dists(nodeId) - 0.1932;
      ekf_iod_B.distsCalibed[arg_nodeId - 1] = arg_dists[arg_nodeId - 1] *
        1.0447 - 0.1932;
    } else {
      // '<S2>:1:7' else
      // '<S2>:1:8' if dists(nodeId) > 1.5 && dists(nodeId) <= 10
      if ((arg_dists[arg_nodeId - 1] > 1.5) && (arg_dists[arg_nodeId - 1] <=
           10.0)) {
        // '<S2>:1:9' distsCalibed(nodeId) = 1.0029 * dists(nodeId) - 0.0829;
        ekf_iod_B.distsCalibed[arg_nodeId - 1] = arg_dists[arg_nodeId - 1] *
          1.0029 - 0.0829;
      } else {
        // '<S2>:1:10' else
        // '<S2>:1:11' distsCalibed(nodeId) = 0.9976 * dists(nodeId) - 0.0511;
        ekf_iod_B.distsCalibed[arg_nodeId - 1] = arg_dists[arg_nodeId - 1] *
          0.9976 - 0.0511;
      }
    }
  }

  // End of MATLAB Function: '<S1>/Range Calibration'

  // MATLAB Function: '<S1>/outlier detection & ekf update' incorporates:
  //   Inport: '<Root>/ancs'
  //   Inport: '<Root>/deltat'
  //   Inport: '<Root>/nodeId'

  // MATLAB Function 'ekf_iod/outlier detection & ekf update': '<S4>:1'
  //  x_pre: 6 by 1, [pos vel]; P_pre: 6 by 6, predicted x and covariance
  //  x_upd: 6 by 1, pos & vel; P_upd: 6 by 6, updated x and covariance;
  //  predicted displacement
  // '<S4>:1:12' disp_pre = x_pre(1:3)' - ancs(:, nodeId)';
  disp_pre_idx_0 = ekf_iod_B.x_pre[0] - arg_ancs[(arg_nodeId - 1) * 3];
  disp_pre_idx_1 = ekf_iod_B.x_pre[1] - arg_ancs[(arg_nodeId - 1) * 3 + 1];
  disp_pre_idx_2 = ekf_iod_B.x_pre[2] - arg_ancs[(arg_nodeId - 1) * 3 + 2];

  //  predicted distance
  // '<S4>:1:14' dist_pre=norm(disp_pre, 2);
  ekf_iod_B.scale = 2.2250738585072014E-308;
  ekf_iod_B.S = fabs(disp_pre_idx_0);
  if (ekf_iod_B.S > 2.2250738585072014E-308) {
    ekf_iod_B.dist_pre = 1.0;
    ekf_iod_B.scale = ekf_iod_B.S;
  } else {
    t = ekf_iod_B.S / 2.2250738585072014E-308;
    ekf_iod_B.dist_pre = t * t;
  }

  ekf_iod_B.S = fabs(disp_pre_idx_1);
  if (ekf_iod_B.S > ekf_iod_B.scale) {
    t = ekf_iod_B.scale / ekf_iod_B.S;
    ekf_iod_B.dist_pre = ekf_iod_B.dist_pre * t * t + 1.0;
    ekf_iod_B.scale = ekf_iod_B.S;
  } else {
    t = ekf_iod_B.S / ekf_iod_B.scale;
    ekf_iod_B.dist_pre += t * t;
  }

  ekf_iod_B.S = fabs(disp_pre_idx_2);
  if (ekf_iod_B.S > ekf_iod_B.scale) {
    t = ekf_iod_B.scale / ekf_iod_B.S;
    ekf_iod_B.dist_pre = ekf_iod_B.dist_pre * t * t + 1.0;
    ekf_iod_B.scale = ekf_iod_B.S;
  } else {
    t = ekf_iod_B.S / ekf_iod_B.scale;
    ekf_iod_B.dist_pre += t * t;
  }

  ekf_iod_B.dist_pre = ekf_iod_B.scale * sqrt(ekf_iod_B.dist_pre);

  // '<S4>:1:15' H=[disp_pre/dist_pre 0 0 0];
  ekf_iod_B.H[0] = disp_pre_idx_0 / ekf_iod_B.dist_pre;
  ekf_iod_B.H[1] = disp_pre_idx_1 / ekf_iod_B.dist_pre;
  ekf_iod_B.H[2] = disp_pre_idx_2 / ekf_iod_B.dist_pre;
  ekf_iod_B.H[3] = 0.0;
  ekf_iod_B.H[4] = 0.0;
  ekf_iod_B.H[5] = 0.0;

  // '<S4>:1:16' innov = dists(nodeId) - dist_pre;
  ekf_iod_B.scale = ekf_iod_B.distsCalibed[arg_nodeId - 1] - ekf_iod_B.dist_pre;

  // '<S4>:1:17' S = R + H*P_pre*H';
  disp_pre_idx_0 = 0.0;
  for (i = 0; i < 6; i++) {
    ekf_iod_B.H_c[i] = 0.0;
    for (i_1 = 0; i_1 < 6; i_1++) {
      ekf_iod_B.H_c[i] += ekf_iod_B.P_pre[6 * i + i_1] * ekf_iod_B.H[i_1];
    }

    disp_pre_idx_0 += ekf_iod_B.H_c[i] * ekf_iod_B.H[i];
  }

  ekf_iod_B.S = ekf_iod_DW.R + disp_pre_idx_0;

  // '<S4>:1:19' outlierDetected = false;
  rtb_outlierDetected = false;

  // '<S4>:1:21' if innov^2/S > ADMIT_RATIO^2
  if (ekf_iod_B.scale * ekf_iod_B.scale / ekf_iod_B.S > ekf_iod_DW.ADMIT_RATIO *
      ekf_iod_DW.ADMIT_RATIO) {
    // '<S4>:1:22' outlierDetected = true;
    rtb_outlierDetected = true;

    // '<S4>:1:23' vmax = zeros(3, 1);
    // '<S4>:1:24' v_prev = x_prev(4:6);
    // '<S4>:1:25' for i = 1:3
    // '<S4>:1:26' vmax(i) = max(abs(v_prev(i)-sqrt(P_prev(i,i))), abs(v_prev(i)+sqrt(P_prev(i,i)))); 
    disp_pre_idx_0 = fabs(ekf_iod_DW.x_prev[3] - sqrt(ekf_iod_DW.P_prev[0]));
    disp_pre_idx_1 = fabs(ekf_iod_DW.x_prev[3] + sqrt(ekf_iod_DW.P_prev[0]));
    if ((disp_pre_idx_0 >= disp_pre_idx_1) || rtIsNaN(disp_pre_idx_1)) {
      disp_pre_idx_1 = disp_pre_idx_0;
    }

    // '<S4>:1:26' vmax(i) = max(abs(v_prev(i)-sqrt(P_prev(i,i))), abs(v_prev(i)+sqrt(P_prev(i,i)))); 
    disp_pre_idx_0 = fabs(ekf_iod_DW.x_prev[4] - sqrt(ekf_iod_DW.P_prev[7]));
    disp_pre_idx_2 = fabs(ekf_iod_DW.x_prev[4] + sqrt(ekf_iod_DW.P_prev[7]));
    if ((disp_pre_idx_0 >= disp_pre_idx_2) || rtIsNaN(disp_pre_idx_2)) {
      disp_pre_idx_2 = disp_pre_idx_0;
    }

    // '<S4>:1:26' vmax(i) = max(abs(v_prev(i)-sqrt(P_prev(i,i))), abs(v_prev(i)+sqrt(P_prev(i,i)))); 
    disp_pre_idx_0 = fabs(ekf_iod_DW.x_prev[5] - sqrt(ekf_iod_DW.P_prev[14]));
    t = fabs(ekf_iod_DW.x_prev[5] + sqrt(ekf_iod_DW.P_prev[14]));
    if ((disp_pre_idx_0 >= t) || rtIsNaN(t)) {
      t = disp_pre_idx_0;
    }

    // '<S4>:1:28' Q = Q + diag([vmax.^2 * deltat^2; zeros(3,1)]);
    ekf_iod_B.scale = arg_deltat * arg_deltat;
    ekf_iod_B.K[0] = disp_pre_idx_1 * disp_pre_idx_1 * ekf_iod_B.scale;
    ekf_iod_B.K[1] = disp_pre_idx_2 * disp_pre_idx_2 * ekf_iod_B.scale;
    ekf_iod_B.K[2] = t * t * ekf_iod_B.scale;
    ekf_iod_B.K[3] = 0.0;
    ekf_iod_B.K[4] = 0.0;
    ekf_iod_B.K[5] = 0.0;
    memset(&ekf_iod_B.A[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      ekf_iod_B.A[i + 6 * i] = ekf_iod_B.K[i];
    }

    // Current prediction is the same but covariance is increased to account
    // for uncertainty of position after a longer time
    // '<S4>:1:32' x_upd = x_prev;
    for (i = 0; i < 6; i++) {
      ekf_iod_B.x_pre[i] = ekf_iod_DW.x_prev[i];
    }

    // '<S4>:1:33' P_upd = P_prev + Q;
    for (i = 0; i < 36; i++) {
      ekf_iod_DW.P_prev[i] += ekf_iod_B.Q[i] + ekf_iod_B.A[i];
    }
  } else {
    // '<S4>:1:34' else
    //  if outlierDetected
    //      %As the update phase detected an outlier, the update phase shall not  
    //      %take the distance in account
    //      x_upd = x_pre;
    //      P_upd = P_pre;
    //  else
    // '<S4>:1:42' K = P_pre*H'/S;
    for (i = 0; i < 6; i++) {
      disp_pre_idx_0 = 0.0;
      for (i_1 = 0; i_1 < 6; i_1++) {
        disp_pre_idx_0 += ekf_iod_B.P_pre[6 * i_1 + i] * ekf_iod_B.H[i_1];
      }

      ekf_iod_B.K[i] = disp_pre_idx_0 / ekf_iod_B.S;
    }

    // '<S4>:1:43' x_upd = x_pre + K*(dists(nodeId) - dist_pre);
    ekf_iod_B.scale = ekf_iod_B.distsCalibed[arg_nodeId - 1] -
      ekf_iod_B.dist_pre;
    for (i = 0; i < 6; i++) {
      ekf_iod_B.x_pre[i] += ekf_iod_B.K[i] * ekf_iod_B.scale;
    }

    // '<S4>:1:44' P_upd = (eye(6) - K*H)*P_pre;
    memset(&ekf_iod_B.A[0], 0, 36U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      ekf_iod_B.A[i + 6 * i] = 1.0;
    }

    for (i = 0; i < 6; i++) {
      for (i_1 = 0; i_1 < 6; i_1++) {
        ekf_iod_B.A_m[i + 6 * i_1] = ekf_iod_B.A[6 * i_1 + i] - ekf_iod_B.K[i] *
          ekf_iod_B.H[i_1];
      }
    }

    for (i = 0; i < 6; i++) {
      for (i_1 = 0; i_1 < 6; i_1++) {
        ekf_iod_DW.P_prev[i + 6 * i_1] = 0.0;
        for (i_0 = 0; i_0 < 6; i_0++) {
          ekf_iod_DW.P_prev[i + 6 * i_1] += ekf_iod_B.A_m[6 * i_0 + i] *
            ekf_iod_B.P_pre[6 * i_1 + i_0];
        }
      }
    }

    // '<S4>:1:45' P_upd = .5*(P_upd + P_upd');
    for (i = 0; i < 6; i++) {
      for (i_1 = 0; i_1 < 6; i_1++) {
        ekf_iod_B.A[i_1 + 6 * i] = (ekf_iod_DW.P_prev[6 * i + i_1] +
          ekf_iod_DW.P_prev[6 * i_1 + i]) * 0.5;
      }
    }

    for (i = 0; i < 6; i++) {
      for (i_1 = 0; i_1 < 6; i_1++) {
        ekf_iod_DW.P_prev[i_1 + 6 * i] = ekf_iod_B.A[6 * i + i_1];
      }
    }

    //  end
  }

  // Outport: '<Root>/outlierDetected'
  // save the current state values for reference in the next loop.
  // '<S4>:1:50' x_prev = x_upd;
  // '<S4>:1:51' P_prev = P_upd;
  *arg_outlierDetected = rtb_outlierDetected;
  for (i = 0; i < 6; i++) {
    // MATLAB Function: '<S1>/outlier detection & ekf update'
    ekf_iod_DW.x_prev[i] = ekf_iod_B.x_pre[i];

    // Outport: '<Root>/x_est'
    arg_x_est[i] = ekf_iod_B.x_pre[i];

    // Update for UnitDelay: '<S1>/Unit Delay1'
    ekf_iod_DW.UnitDelay1_DSTATE[i] = ekf_iod_B.x_pre[i];
  }

  // Update for UnitDelay: '<S1>/Unit Delay2' incorporates:
  //   MATLAB Function: '<S1>/outlier detection & ekf update'

  memcpy(&ekf_iod_DW.UnitDelay2_DSTATE[0], &ekf_iod_DW.P_prev[0], 36U * sizeof
         (real_T));
}

// Model initialize function
void ekfIodClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    int32_T i;

    // Start for DataStoreMemory: '<S1>/ADMIT_RATIO'
    ekf_iod_DW.ADMIT_RATIO = ekf_iod_P.ADMIT_RATIO_InitialValue;

    // Start for DataStoreMemory: '<S1>/P_prev'
    memcpy(&ekf_iod_DW.P_prev[0], &ekf_iod_P.P_0[0], 36U * sizeof(real_T));

    // Start for DataStoreMemory: '<S1>/R'
    ekf_iod_DW.R = ekf_iod_P.R_InitialValue;

    // Start for DataStoreMemory: '<S1>/acc_xy'
    ekf_iod_DW.acc_xy = ekf_iod_P.acc_xy_InitialValue;

    // Start for DataStoreMemory: '<S1>/acc_z'
    ekf_iod_DW.acc_z = ekf_iod_P.acc_z_InitialValue;

    // Start for DataStoreMemory: '<S1>/x_prev'
    for (i = 0; i < 6; i++) {
      ekf_iod_DW.x_prev[i] = ekf_iod_P.ekf_iod_x_hat0[i];
    }

    // End of Start for DataStoreMemory: '<S1>/x_prev'
  }

  {
    int32_T i;

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay1'
    for (i = 0; i < 6; i++) {
      ekf_iod_DW.UnitDelay1_DSTATE[i] = ekf_iod_P.ekf_iod_x_hat0[i];
    }

    // End of InitializeConditions for UnitDelay: '<S1>/Unit Delay1'

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay2'
    memcpy(&ekf_iod_DW.UnitDelay2_DSTATE[0], &ekf_iod_P.P_0[0], 36U * sizeof
           (real_T));
  }
}

// Model terminate function
void ekfIodClass::terminate()
{
  // (no terminate code required)
}

// Constructor
ekfIodClass::ekfIodClass()
{
  P_ekf_iod_T ekf_iod_P_temp = {
    //  Variable: P_0
    //  Referenced by:
    //    '<S1>/P_prev'
    //    '<S1>/Unit Delay2'

    { 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

    //  Mask Parameter: ekf_iod_x_hat0
    //  Referenced by:
    //    '<S1>/x_prev'
    //    '<S1>/Unit Delay1'

    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
    1.5,                               // Expression: 1.5
                                       //  Referenced by: '<S1>/ADMIT_RATIO'

    0.2,                               // Expression: 0.2
                                       //  Referenced by: '<S1>/R'

    5.0,                               // Expression: 5
                                       //  Referenced by: '<S1>/acc_xy'

    2.0                                // Expression: 2
                                       //  Referenced by: '<S1>/acc_z'

  };                                   // Modifiable parameters

  // Initialize tunable parameters
  ekf_iod_P = ekf_iod_P_temp;
}

// Destructor
ekfIodClass::~ekfIodClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_ekf_iod_T * ekfIodClass::getRTM()
{
  return (&ekf_iod_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
