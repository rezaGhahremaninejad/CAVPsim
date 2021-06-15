/*
 * vehicleModel.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "vehicleModel".
 *
 * Model version              : 1.8
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue Jun 15 08:09:38 2021
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "vehicleModel.h"
#include "vehicleModel_private.h"

/* Forward declaration for local functions */
static void BicycleKinematics_assignModelPr(robotics_slmobile_internal_bl_T *obj);
static void vehicleModel_SystemCore_setup(robotics_slmobile_internal_bl_T *obj);
real_T look1_binlcpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si ,
  RT_MODEL_vehicleModel_T *const vehicleModel_M)
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 7;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  vehicleModel_derivatives(vehicleModel_M);

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  vehicleModel_step(vehicleModel_M);
  vehicleModel_derivatives(vehicleModel_M);

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  vehicleModel_step(vehicleModel_M);
  vehicleModel_derivatives(vehicleModel_M);

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

static void BicycleKinematics_assignModelPr(robotics_slmobile_internal_bl_T *obj)
{
  char_T b_f1[25];
  char_T vehicleInputs_data[25];
  d_cell_wrap_vehicleModel_T f;
  d_cell_wrap_vehicleModel_T g;
  char_T partial_match_data[25];
  int32_T nmatched;
  boolean_T b_bool;
  int32_T kstr;
  int32_T b_kstr;
  char_T b[25];
  static const char_T tmp[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l',
    'e' };

  static const char_T tmp_0[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '	', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e',
    '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17',
    '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!',
    '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']',
    '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
    'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{',
    '|', '}', '~', '\x7f' };

  static const char_T tmp_1[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  int32_T exitg1;
  for (kstr = 0; kstr < 25; kstr++) {
    b_f1[kstr] = tmp[kstr];
    b[kstr] = tmp[kstr];
  }

  b_bool = false;
  b_kstr = 1;
  do {
    exitg1 = 0;
    if (b_kstr - 1 < 25) {
      kstr = b_kstr - 1;
      if (tmp_0[(int32_T)b_f1[kstr]] != tmp_0[(int32_T)b[kstr]]) {
        exitg1 = 1;
      } else {
        b_kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (b_bool) {
    nmatched = 1;
    b_kstr = 25;
    for (kstr = 0; kstr < 25; kstr++) {
      partial_match_data[kstr] = tmp[kstr];
    }
  } else {
    nmatched = 0;
    b_kstr = 0;
  }

  if ((nmatched == 0) || (b_kstr == 0)) {
    b_kstr = 0;
  } else {
    kstr = b_kstr - 1;
    if (0 <= kstr) {
      memcpy(&vehicleInputs_data[0], &partial_match_data[0], (kstr + 1) * sizeof
             (char_T));
    }
  }

  for (kstr = 0; kstr < 25; kstr++) {
    b[kstr] = tmp[kstr];
  }

  b_bool = false;
  if (b_kstr == 25) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 25) {
        kstr = b_kstr - 1;
        if (vehicleInputs_data[kstr] != b[kstr]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  for (kstr = 0; kstr < 25; kstr++) {
    if (b_bool) {
      f.f1[kstr] = tmp[kstr];
      obj->KinModel.VehicleInputsInternal[kstr] = f.f1[kstr];
    } else {
      g.f1[kstr] = tmp_1[kstr];
      obj->KinModel.VehicleInputsInternal[kstr] = g.f1[kstr];
    }
  }

  obj->KinModel.WheelBase = obj->WheelBase;
  obj->KinModel.VehicleSpeedRange[0] = obj->VehicleSpeedRange[0];
  obj->KinModel.VehicleSpeedRange[1] = obj->VehicleSpeedRange[1];
  obj->KinModel.MaxSteeringAngle = obj->MaxSteeringAngle;
}

static void vehicleModel_SystemCore_setup(robotics_slmobile_internal_bl_T *obj)
{
  char_T b_f4[25];
  char_T vehicleInputs_data[25];
  d_cell_wrap_vehicleModel_T f;
  d_cell_wrap_vehicleModel_T g;
  char_T partial_match_data[25];
  int32_T nmatched;
  boolean_T b_bool;
  int32_T kstr;
  int32_T b_kstr;
  char_T b[25];
  static const char_T tmp[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l',
    'e' };

  static const char_T tmp_0[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '	', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e',
    '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17',
    '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!',
    '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']',
    '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
    'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{',
    '|', '}', '~', '\x7f' };

  static const char_T tmp_1[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  int32_T exitg1;
  obj->isInitialized = 1;
  obj->KinModel.WheelBase = 1.0;
  obj->KinModel.VehicleSpeedRange[0] = (rtMinusInf);
  obj->KinModel.VehicleSpeedRange[1] = (rtInf);
  obj->KinModel.MaxSteeringAngle = 0.78539816339744828;
  for (kstr = 0; kstr < 25; kstr++) {
    b_f4[kstr] = tmp[kstr];
    b[kstr] = tmp[kstr];
  }

  b_bool = false;
  b_kstr = 1;
  do {
    exitg1 = 0;
    if (b_kstr - 1 < 25) {
      kstr = b_kstr - 1;
      if (tmp_0[(int32_T)b_f4[kstr]] != tmp_0[(int32_T)b[kstr]]) {
        exitg1 = 1;
      } else {
        b_kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (b_bool) {
    nmatched = 1;
    b_kstr = 25;
    for (kstr = 0; kstr < 25; kstr++) {
      partial_match_data[kstr] = tmp[kstr];
    }
  } else {
    nmatched = 0;
    b_kstr = 0;
  }

  if ((nmatched == 0) || (b_kstr == 0)) {
    b_kstr = 0;
  } else {
    kstr = b_kstr - 1;
    if (0 <= kstr) {
      memcpy(&vehicleInputs_data[0], &partial_match_data[0], (kstr + 1) * sizeof
             (char_T));
    }
  }

  for (kstr = 0; kstr < 25; kstr++) {
    b[kstr] = tmp[kstr];
  }

  b_bool = false;
  if (b_kstr == 25) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 25) {
        kstr = b_kstr - 1;
        if (vehicleInputs_data[kstr] != b[kstr]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  for (kstr = 0; kstr < 25; kstr++) {
    if (b_bool) {
      f.f1[kstr] = tmp[kstr];
      obj->KinModel.VehicleInputsInternal[kstr] = f.f1[kstr];
    } else {
      g.f1[kstr] = tmp_1[kstr];
      obj->KinModel.VehicleInputsInternal[kstr] = g.f1[kstr];
    }
  }

  BicycleKinematics_assignModelPr(obj);
  obj->TunablePropsChanged = false;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void vehicleModel_step(RT_MODEL_vehicleModel_T *const vehicleModel_M)
{
  B_vehicleModel_T *vehicleModel_B = ((B_vehicleModel_T *)
    vehicleModel_M->blockIO);
  DW_vehicleModel_T *vehicleModel_DW = ((DW_vehicleModel_T *)
    vehicleModel_M->dwork);
  X_vehicleModel_T *vehicleModel_X = ((X_vehicleModel_T *)
    vehicleModel_M->contStates);
  ExtU_vehicleModel_T *vehicleModel_U = (ExtU_vehicleModel_T *)
    vehicleModel_M->inputs;
  ExtY_vehicleModel_T *vehicleModel_Y = (ExtY_vehicleModel_T *)
    vehicleModel_M->outputs;
  boolean_T flag;
  real_T varargin_1[2];
  robotics_slmobile_internal_bl_T *obj;
  boolean_T p;
  char_T switch_expression[25];
  int32_T ret;
  char_T b[25];
  real_T rtb_Transpose1[9];
  real_T rtb_sincos_o1_f[3];
  real_T rtb_Add_e;
  real_T rtb_VectorConcatenate_m[9];
  real_T rtb_sincos_o2[3];
  real_T rtb_VectorConcatenate[9];
  real_T rtb_Add_n;
  real_T rtb_Product_b;
  real_T rtb_Add6;
  real_T rtb_Product2_h;
  real_T rtb_Product1_f;
  real_T rtb_uAPabsRT[6];
  real_T rtb_VectorConcatenate5[3];
  real_T rtb_Product2;
  real_T rtb_VectorConcatenate4_idx_2;
  real_T rtb_VectorConcatenate5_b_idx_2;
  real_T rtb_TmpSignalConversionAtProd_0;
  real_T rtb_Product4_idx_1;
  real_T rtb_VectorConcatenate6_idx_0;
  real_T rtb_VectorConcatenate6_idx_2;
  real_T rtb_sincos_o1_idx_2;
  real_T rtb_sincos_o1_idx_1;
  real_T rtb_sincos_o1_idx_0;
  real_T rtb_Product8_idx_0;
  real_T rtb_Product8_idx_1;
  real_T rtb_VectorConcatenate_tmp;
  real_T rtb_VectorConcatenate_i_tmp;
  static const char_T tmp[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'S', 't', 'e', 'e', 'r', 'i', 'n', 'g', 'A', 'n', 'g', 'l',
    'e' };

  static const char_T tmp_0[25] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e', '-',
    '-' };

  boolean_T exitg1;
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* set solver stop time */
    if (!(vehicleModel_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(vehicleModel_M->solverInfo,
                            ((vehicleModel_M->Timing.clockTickH0 + 1) *
        vehicleModel_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(vehicleModel_M->solverInfo,
                            ((vehicleModel_M->Timing.clockTick0 + 1) *
        vehicleModel_M->Timing.stepSize0 + vehicleModel_M->Timing.clockTickH0 *
        vehicleModel_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(vehicleModel_M)) {
    vehicleModel_M->Timing.t[0] = rtsiGetT(vehicleModel_M->solverInfo);
  }

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Integrator: '<S7>/Integrator' */
  vehicleModel_B->Integrator = vehicleModel_X->Integrator_CSTATE_b;

  /* SignalConversion generated from: '<S4>/Vector Concatenate2' */
  vehicleModel_B->VectorConcatenate2[0] = vehicleModel_B->Integrator;

  /* SignalConversion generated from: '<S4>/Vector Concatenate2' */
  vehicleModel_B->VectorConcatenate2[1] = 0.0;

  /* SignalConversion generated from: '<S4>/Vector Concatenate2' */
  vehicleModel_B->VectorConcatenate2[2] = 0.0;

  /* MATLABSystem: '<S2>/MATLAB System' incorporates:
   *  Integrator: '<S1>/Integrator'
   *  Integrator: '<S2>/Integrator'
   */
  if (vehicleModel_DW->obj.WheelBase !=
      vehicleModel_P.BicycleKinematicModel_WheelBase) {
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[0] = true;
    }

    vehicleModel_DW->obj.WheelBase =
      vehicleModel_P.BicycleKinematicModel_WheelBase;
  }

  varargin_1[0] = vehicleModel_DW->obj.VehicleSpeedRange[0];
  varargin_1[1] = vehicleModel_DW->obj.VehicleSpeedRange[1];
  flag = false;
  p = true;
  ret = 0;
  exitg1 = false;
  while ((!exitg1) && (ret < 2)) {
    if (!(varargin_1[ret] == vehicleModel_P.BicycleKinematicModel_VehicleSp[ret]))
    {
      p = false;
      exitg1 = true;
    } else {
      ret++;
    }
  }

  if (p) {
    flag = true;
  }

  if (!flag) {
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[1] = true;
    }

    vehicleModel_DW->obj.VehicleSpeedRange[0] =
      vehicleModel_P.BicycleKinematicModel_VehicleSp[0];
    vehicleModel_DW->obj.VehicleSpeedRange[1] =
      vehicleModel_P.BicycleKinematicModel_VehicleSp[1];
  }

  if (vehicleModel_DW->obj.MaxSteeringAngle !=
      vehicleModel_P.BicycleKinematicModel_MaxSteeri) {
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[2] = true;
    }

    vehicleModel_DW->obj.MaxSteeringAngle =
      vehicleModel_P.BicycleKinematicModel_MaxSteeri;
  }

  obj = &vehicleModel_DW->obj;
  if (vehicleModel_DW->obj.TunablePropsChanged) {
    vehicleModel_DW->obj.TunablePropsChanged = false;
    flag = vehicleModel_DW->obj.tunablePropertyChanged[1];
    if (flag) {
      varargin_1[0] = vehicleModel_DW->obj.VehicleSpeedRange[0];
      varargin_1[1] = vehicleModel_DW->obj.VehicleSpeedRange[1];
      obj->KinModel.VehicleSpeedRange[0] = varargin_1[0];
      obj->KinModel.VehicleSpeedRange[1] = varargin_1[1];
    }

    flag = vehicleModel_DW->obj.tunablePropertyChanged[0];
    if (flag) {
      rtb_Product1_f = vehicleModel_DW->obj.WheelBase;
      obj->KinModel.WheelBase = rtb_Product1_f;
    }

    flag = vehicleModel_DW->obj.tunablePropertyChanged[2];
    if (flag) {
      rtb_Product1_f = vehicleModel_DW->obj.MaxSteeringAngle;
      obj->KinModel.MaxSteeringAngle = rtb_Product1_f;
    }

    vehicleModel_DW->obj.tunablePropertyChanged[0] = false;
    vehicleModel_DW->obj.tunablePropertyChanged[1] = false;
    vehicleModel_DW->obj.tunablePropertyChanged[2] = false;
  }

  rtb_Product1_f = obj->KinModel.VehicleSpeedRange[0];
  rtb_Add_e = obj->KinModel.VehicleSpeedRange[1];
  rtb_Product1_f = fmin(fmax(vehicleModel_B->VectorConcatenate2[0],
    rtb_Product1_f), rtb_Add_e);
  rtb_Add_e = 0.0;
  for (ret = 0; ret < 25; ret++) {
    switch_expression[ret] = obj->KinModel.VehicleInputsInternal[ret];
  }

  for (ret = 0; ret < 25; ret++) {
    b[ret] = tmp[ret];
  }

  ret = memcmp(&switch_expression[0], &b[0], 25);
  if (ret == 0) {
    rtb_Product2 = 0.0;
  } else {
    for (ret = 0; ret < 25; ret++) {
      b[ret] = tmp_0[ret];
    }

    ret = memcmp(&switch_expression[0], &b[0], 25);
    if (ret == 0) {
      rtb_Product2 = 1.0;
    } else {
      rtb_Product2 = -1.0;
    }
  }

  switch ((int32_T)rtb_Product2) {
   case 0:
    rtb_Add_e = vehicleModel_X->Integrator_CSTATE_l;
    if (fabs(vehicleModel_X->Integrator_CSTATE_l) >
        obj->KinModel.MaxSteeringAngle) {
      if (vehicleModel_X->Integrator_CSTATE_l < 0.0) {
        rtb_Add_n = -1.0;
      } else if (vehicleModel_X->Integrator_CSTATE_l > 0.0) {
        rtb_Add_n = 1.0;
      } else if (vehicleModel_X->Integrator_CSTATE_l == 0.0) {
        rtb_Add_n = 0.0;
      } else {
        rtb_Add_n = (rtNaN);
      }

      rtb_Add_e = rtb_Add_n * obj->KinModel.MaxSteeringAngle;
    }

    rtb_Add_e = rtb_Product1_f / obj->KinModel.WheelBase * tan(rtb_Add_e);
    break;

   case 1:
    rtb_Product2_h = vehicleModel_X->Integrator_CSTATE_l *
      obj->KinModel.WheelBase;
    rtb_Add_e = rt_atan2d_snf(rtb_Product2_h, vehicleModel_B->
      VectorConcatenate2[0]);
    if (fabs(rtb_Add_e) > obj->KinModel.MaxSteeringAngle) {
      if (rtb_Add_e < 0.0) {
        rtb_Add_e = -1.0;
      } else if (rtb_Add_e > 0.0) {
        rtb_Add_e = 1.0;
      } else if (rtb_Add_e == 0.0) {
        rtb_Add_e = 0.0;
      } else {
        rtb_Add_e = (rtNaN);
      }

      rtb_Add_e *= obj->KinModel.MaxSteeringAngle;
    }

    rtb_Add_e = vehicleModel_B->VectorConcatenate2[0] / obj->KinModel.WheelBase *
      tan(rtb_Add_e);
    rtb_Product1_f = vehicleModel_B->VectorConcatenate2[0];
    break;
  }

  rtb_uAPabsRT[0] = cos(vehicleModel_X->Integrator_CSTATE[2]);
  rtb_uAPabsRT[3] = 0.0;
  rtb_uAPabsRT[1] = sin(vehicleModel_X->Integrator_CSTATE[2]);
  rtb_uAPabsRT[4] = 0.0;
  rtb_uAPabsRT[2] = 0.0;
  rtb_uAPabsRT[5] = 1.0;
  for (ret = 0; ret < 3; ret++) {
    vehicleModel_B->MATLABSystem[ret] = 0.0;
    vehicleModel_B->MATLABSystem[ret] += rtb_uAPabsRT[ret] * rtb_Product1_f;
    vehicleModel_B->MATLABSystem[ret] += rtb_uAPabsRT[ret + 3] * rtb_Add_e;
  }

  /* End of MATLABSystem: '<S2>/MATLAB System' */

  /* UnitConversion: '<S4>/Unit Conversion' incorporates:
   *  Inport: '<Root>/Grade'
   */
  /* Unit Conversion - from: deg to: rad
     Expression: output = (0.0174533*input) + (0) */
  rtb_Add_e = 0.017453292519943295 * vehicleModel_U->Grade;
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* Gain: '<S7>/m' incorporates:
     *  Constant: '<S7>/g'
     */
    vehicleModel_B->Fz = vehicleModel_P.VehicleBody1DOFLongitudinal_m *
      vehicleModel_P.VehicleBody1DOFLongitudinal_g;

    /* UnaryMinus: '<S44>/Unary Minus' incorporates:
     *  Constant: '<S44>/Constant4'
     */
    vehicleModel_B->UnaryMinus[0] = -vehicleModel_P.Constant4_Value[0];
    vehicleModel_B->UnaryMinus[1] = -vehicleModel_P.Constant4_Value[1];
    vehicleModel_B->UnaryMinus[2] = -vehicleModel_P.Constant4_Value[2];
  }

  /* Trigonometry: '<S7>/Trigonometric Function2' incorporates:
   *  Trigonometry: '<S35>/sincos'
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_Product8_idx_0 = sin(rtb_Add_e);

  /* Product: '<S7>/Product1' incorporates:
   *  Trigonometry: '<S7>/Trigonometric Function2'
   */
  rtb_Product1_f = rtb_Product8_idx_0 * vehicleModel_B->Fz;

  /* Sum: '<S44>/Add1' incorporates:
   *  Inport: '<Root>/WindX'
   *  SignalConversion generated from: '<S7>/Vector Concatenate5'
   *  UnaryMinus: '<S7>/Unary Minus'
   */
  rtb_Product2_h = vehicleModel_B->Integrator - (-vehicleModel_U->WindX);

  /* Switch: '<S44>/Switch' incorporates:
   *  Constant: '<S44>/Constant4'
   */
  if (rtb_Product2_h >= vehicleModel_P.Switch_Threshold) {
    rtb_TmpSignalConversionAtProd_0 = vehicleModel_P.Constant4_Value[0];
  } else {
    rtb_TmpSignalConversionAtProd_0 = vehicleModel_B->UnaryMinus[0];
  }

  /* Sqrt: '<S44>/Sqrt' incorporates:
   *  Product: '<S44>/Product'
   */
  rtb_Add_n = sqrt(rtb_Product2_h * rtb_Product2_h);

  /* Product: '<S44>/Product2' */
  rtb_Product2 = rtb_Add_n * rtb_Add_n;
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* Constant: '<S44>/Constant' */
    vehicleModel_B->VectorConcatenate[0] =
      vehicleModel_P.VehicleBody1DOFLongitudinal_Cd;
  }

  /* Trigonometry: '<S44>/Trigonometric Function' */
  rtb_Add_n = rt_atan2d_snf(0.0, rtb_Product2_h);

  /* Lookup_n-D: '<S44>/Cs' */
  vehicleModel_B->VectorConcatenate[1] = look1_binlcpw(rtb_Add_n,
    vehicleModel_P.DragForce_beta_w, vehicleModel_P.DragForce_Cs, 1U);
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* Constant: '<S44>/Constant1' */
    vehicleModel_B->VectorConcatenate[2] =
      vehicleModel_P.VehicleBody1DOFLongitudinal_Cl;
  }

  /* Lookup_n-D: '<S44>/Crm' */
  vehicleModel_B->VectorConcatenate[3] = look1_binlxpw(rtb_Add_n,
    vehicleModel_P.Crm_bp01Data, vehicleModel_P.Crm_tableData, 1U);

  /* Product: '<S44>/Product5' incorporates:
   *  Constant: '<S44>/Constant2'
   */
  vehicleModel_B->VectorConcatenate[4] = rtb_TmpSignalConversionAtProd_0 *
    vehicleModel_P.VehicleBody1DOFLongitudinal_Cpm;

  /* Lookup_n-D: '<S44>/Cym' */
  vehicleModel_B->VectorConcatenate[5] = look1_binlxpw(rtb_Add_n,
    vehicleModel_P.DragForce_beta_w, vehicleModel_P.DragForce_Cym, 1U);

  /* Gain: '<S44>/.5.*A.*Pabs.//R.//T' incorporates:
   *  Constant: '<S3>/AirTempConstant'
   *  Product: '<S44>/Product1'
   */
  rtb_Product2_h = 0.5 * vehicleModel_P.VehicleBody1DOFLongitudinal_Af *
    vehicleModel_P.VehicleBody1DOFLongitudinal_Pab / vehicleModel_P.DragForce_R;
  for (ret = 0; ret < 6; ret++) {
    rtb_uAPabsRT[ret] = rtb_Product2 * vehicleModel_B->VectorConcatenate[ret] /
      vehicleModel_P.VehicleBody1DOFLongitudinal_T * rtb_Product2_h;
  }

  /* End of Gain: '<S44>/.5.*A.*Pabs.//R.//T' */

  /* Product: '<S44>/Product3' */
  rtb_TmpSignalConversionAtProd_0 *= rtb_uAPabsRT[0];

  /* Switch: '<S44>/Switch' incorporates:
   *  Constant: '<S44>/Constant4'
   */
  if (0.0 >= vehicleModel_P.Switch_Threshold) {
    rtb_Add_n = vehicleModel_P.Constant4_Value[2];
  } else {
    rtb_Add_n = vehicleModel_B->UnaryMinus[2];
  }

  /* Product: '<S44>/Product3' */
  rtb_VectorConcatenate5_b_idx_2 = rtb_Add_n * rtb_uAPabsRT[2];

  /* Sum: '<S7>/Add' incorporates:
   *  Constant: '<S3>/FExtConstant'
   *  Inport: '<Root>/FwF'
   *  Inport: '<Root>/FwR'
   */
  rtb_Add_n = (((vehicleModel_P.FExtConstant_Value[0] - rtb_Product1_f) +
                vehicleModel_U->FwF) + vehicleModel_U->FwR) -
    rtb_TmpSignalConversionAtProd_0;

  /* Gain: '<S7>/1//m' */
  vehicleModel_B->xddot = 1.0 / vehicleModel_P.VehicleBody1DOFLongitudinal_m *
    rtb_Add_n;

  /* UnitConversion: '<S15>/Unit Conversion' */
  /* Unit Conversion - from: m/s^2 to: gn
     Expression: output = (0.101972*input) + (0) */
  rtb_Product2 = 0.10197162129779282 * vehicleModel_B->xddot;

  /* Product: '<S44>/Product4' incorporates:
   *  Constant: '<S44>/Constant3'
   *  Gain: '<S7>/1//(a+b)'
   */
  rtb_VectorConcatenate4_idx_2 = vehicleModel_P.VehicleBody1DOFLongitudinal_a +
    vehicleModel_P.VehicleBody1DOFLongitudinal_b;

  /* Outport: '<Root>/Info' incorporates:
   *  Constant: '<S44>/Constant3'
   *  Product: '<S44>/Product4'
   */
  vehicleModel_Y->Info[63] = rtb_uAPabsRT[3] * rtb_VectorConcatenate4_idx_2;

  /* Product: '<S44>/Product4' incorporates:
   *  Constant: '<S44>/Constant3'
   */
  rtb_Product4_idx_1 = rtb_uAPabsRT[4] * rtb_VectorConcatenate4_idx_2;

  /* Outport: '<Root>/Info' incorporates:
   *  Constant: '<S44>/Constant3'
   *  Product: '<S44>/Product4'
   */
  vehicleModel_Y->Info[65] = rtb_uAPabsRT[5] * rtb_VectorConcatenate4_idx_2;

  /* Sum: '<S7>/Add6' incorporates:
   *  Constant: '<S3>/MExtConstant'
   */
  rtb_Add6 = rtb_Product4_idx_1 + vehicleModel_P.MExtConstant_Value[1];

  /* Trigonometry: '<S7>/Trigonometric Function2' incorporates:
   *  Trigonometry: '<S35>/sincos'
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_Product8_idx_1 = cos(rtb_Add_e);

  /* Product: '<S7>/Product2' incorporates:
   *  Trigonometry: '<S7>/Trigonometric Function2'
   */
  rtb_Product2_h = rtb_Product8_idx_1 * vehicleModel_B->Fz;

  /* Sum: '<S7>/Add1' incorporates:
   *  Constant: '<S3>/FExtConstant'
   */
  rtb_Add_e = (rtb_Product2_h + vehicleModel_P.FExtConstant_Value[2]) +
    rtb_VectorConcatenate5_b_idx_2;

  /* Gain: '<S7>/h' incorporates:
   *  Constant: '<S3>/FExtConstant'
   *  Sum: '<S7>/Add3'
   */
  rtb_Product_b = (((rtb_TmpSignalConversionAtProd_0 -
                     vehicleModel_P.FExtConstant_Value[0]) + rtb_Product1_f) +
                   rtb_Add_n) * vehicleModel_P.VehicleBody1DOFLongitudinal_h;

  /* Gain: '<S7>/1//(a+b)' incorporates:
   *  Gain: '<S7>/1//(a+b) '
   *  Gain: '<S7>/b'
   *  Sum: '<S7>/Add2'
   */
  rtb_VectorConcatenate6_idx_0 = 1.0 / rtb_VectorConcatenate4_idx_2;
  rtb_VectorConcatenate4_idx_2 = ((vehicleModel_P.VehicleBody1DOFLongitudinal_b *
    rtb_Add_e - rtb_Add6) - rtb_Product_b) * rtb_VectorConcatenate6_idx_0;

  /* Gain: '<S7>/1//(a+b) ' incorporates:
   *  Gain: '<S7>/a'
   *  Sum: '<S7>/Add4'
   */
  rtb_Product_b = ((vehicleModel_P.VehicleBody1DOFLongitudinal_a * rtb_Add_e +
                    rtb_Add6) + rtb_Product_b) * rtb_VectorConcatenate6_idx_0;

  /* Gain: '<S16>/1//NF' incorporates:
   *  Selector: '<S16>/Selector1'
   */
  rtb_Add6 = 1.0 / vehicleModel_P.VehicleBody1DOFLongitudinal_NF *
    rtb_VectorConcatenate4_idx_2;

  /* Gain: '<S16>/1//NR' incorporates:
   *  Selector: '<S16>/Selector1'
   */
  rtb_Add_e = 1.0 / vehicleModel_P.VehicleBody1DOFLongitudinal_NR *
    rtb_Product_b;

  /* Trigonometry: '<S42>/sincos' incorporates:
   *  Constant: '<S6>/Constant'
   */
  rtb_sincos_o1_idx_0 = cos(vehicleModel_P.Constant_Value);
  rtb_sincos_o1_idx_1 = sin(vehicleModel_P.Constant_Value);

  /* Fcn: '<S42>/Fcn11' incorporates:
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_VectorConcatenate[0] = rtb_Product8_idx_1 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S42>/Fcn21' incorporates:
   *  Fcn: '<S42>/Fcn22'
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_sincos_o1_idx_2 = rtb_sincos_o1_idx_1 * rtb_Product8_idx_0;
  rtb_VectorConcatenate[1] = rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_0 -
    rtb_sincos_o1_idx_0 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S42>/Fcn31' incorporates:
   *  Fcn: '<S42>/Fcn32'
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_VectorConcatenate_tmp = rtb_sincos_o1_idx_0 * rtb_Product8_idx_0;
  rtb_VectorConcatenate[2] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_idx_0 +
    rtb_sincos_o1_idx_1 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S42>/Fcn12' incorporates:
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_VectorConcatenate[3] = rtb_Product8_idx_1 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S42>/Fcn22' incorporates:
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_VectorConcatenate[4] = rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_1 +
    rtb_sincos_o1_idx_0 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S42>/Fcn32' incorporates:
   *  Trigonometry: '<S42>/sincos'
   */
  rtb_VectorConcatenate[5] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_idx_1 -
    rtb_sincos_o1_idx_1 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S42>/Fcn13' */
  rtb_VectorConcatenate[6] = -rtb_Product8_idx_0;

  /* Fcn: '<S42>/Fcn23' */
  rtb_VectorConcatenate[7] = rtb_sincos_o1_idx_1 * rtb_Product8_idx_1;

  /* Fcn: '<S42>/Fcn33' */
  rtb_VectorConcatenate[8] = rtb_sincos_o1_idx_0 * rtb_Product8_idx_1;

  /* Product: '<S6>/Product' incorporates:
   *  Constant: '<S6>/Constant'
   *  Integrator: '<S4>/Integrator1'
   *  Math: '<S6>/Transpose'
   */
  for (ret = 0; ret < 3; ret++) {
    rtb_VectorConcatenate5[ret] = rtb_VectorConcatenate[3 * ret + 2] * 0.0 +
      (rtb_VectorConcatenate[3 * ret + 1] * vehicleModel_P.Constant_Value +
       rtb_VectorConcatenate[3 * ret] * vehicleModel_X->Integrator1_CSTATE);
  }

  /* End of Product: '<S6>/Product' */

  /* SignalConversion generated from: '<S4>/Vector Concatenate6' */
  rtb_VectorConcatenate6_idx_0 = rtb_VectorConcatenate5[0];

  /* SignalConversion generated from: '<S4>/Vector Concatenate6' */
  rtb_VectorConcatenate6_idx_2 = rtb_VectorConcatenate5[2];
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* SignalConversion generated from: '<S20>/sincos' incorporates:
     *  Constant: '<S12>/Constant1'
     *  Constant: '<S12>/Constant2'
     */
    rtb_sincos_o1_f[0] = vehicleModel_P.Constant2_Value;
    rtb_sincos_o1_f[1] = 0.0;
    rtb_sincos_o1_f[2] = vehicleModel_P.Constant1_Value_l;

    /* Trigonometry: '<S20>/sincos' incorporates:
     *  Constant: '<S12>/Constant1'
     *  Constant: '<S12>/Constant2'
     *  SignalConversion generated from: '<S20>/sincos'
     */
    rtb_sincos_o1_idx_0 = sin(vehicleModel_P.Constant2_Value);
    rtb_sincos_o2[0] = cos(vehicleModel_P.Constant2_Value);
    rtb_sincos_o1_idx_1 = sin(0.0);
    rtb_sincos_o2[1] = cos(0.0);
    rtb_sincos_o1_idx_2 = sin(vehicleModel_P.Constant1_Value_l);
    rtb_sincos_o2[2] = cos(vehicleModel_P.Constant1_Value_l);

    /* Fcn: '<S20>/Fcn11' */
    rtb_VectorConcatenate_m[0] = rtb_sincos_o2[1] * rtb_sincos_o2[0];

    /* Fcn: '<S20>/Fcn21' incorporates:
     *  Fcn: '<S20>/Fcn22'
     */
    rtb_VectorConcatenate_tmp = rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_1;
    rtb_VectorConcatenate_m[1] = rtb_VectorConcatenate_tmp * rtb_sincos_o2[0] -
      rtb_sincos_o2[2] * rtb_sincos_o1_idx_0;

    /* Fcn: '<S20>/Fcn31' incorporates:
     *  Fcn: '<S20>/Fcn32'
     */
    rtb_VectorConcatenate_i_tmp = rtb_sincos_o2[2] * rtb_sincos_o1_idx_1;
    rtb_VectorConcatenate_m[2] = rtb_VectorConcatenate_i_tmp * rtb_sincos_o2[0]
      + rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_0;

    /* Fcn: '<S20>/Fcn12' */
    rtb_VectorConcatenate_m[3] = rtb_sincos_o2[1] * rtb_sincos_o1_idx_0;

    /* Fcn: '<S20>/Fcn22' */
    rtb_VectorConcatenate_m[4] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_idx_0
      + rtb_sincos_o2[2] * rtb_sincos_o2[0];

    /* Fcn: '<S20>/Fcn32' */
    rtb_VectorConcatenate_m[5] = rtb_VectorConcatenate_i_tmp *
      rtb_sincos_o1_idx_0 - rtb_sincos_o1_idx_2 * rtb_sincos_o2[0];

    /* Fcn: '<S20>/Fcn13' */
    rtb_VectorConcatenate_m[6] = -rtb_sincos_o1_idx_1;

    /* Fcn: '<S20>/Fcn23' */
    rtb_VectorConcatenate_m[7] = rtb_sincos_o1_idx_2 * rtb_sincos_o2[1];

    /* Fcn: '<S20>/Fcn33' */
    rtb_VectorConcatenate_m[8] = rtb_sincos_o2[2] * rtb_sincos_o2[1];

    /* Math: '<S18>/Transpose1' */
    for (ret = 0; ret < 3; ret++) {
      rtb_Transpose1[3 * ret] = rtb_VectorConcatenate_m[ret];
      rtb_Transpose1[3 * ret + 1] = rtb_VectorConcatenate_m[ret + 3];
      rtb_Transpose1[3 * ret + 2] = rtb_VectorConcatenate_m[ret + 6];
    }

    /* End of Math: '<S18>/Transpose1' */

    /* Constant: '<S18>/R_T1' */
    vehicleModel_B->VectorConcatenate_k[0] =
      vehicleModel_P.VehicleBody1DOFLongitudinal_a;

    /* Constant: '<S18>/R_T2' */
    vehicleModel_B->VectorConcatenate_k[1] =
      vehicleModel_P.HardPointCoordinateTransformFro;

    /* Constant: '<S18>/R_T3' */
    vehicleModel_B->VectorConcatenate_k[2] =
      vehicleModel_P.VehicleBody1DOFLongitudinal_h;

    /* Product: '<S22>/Product' */
    for (ret = 0; ret < 3; ret++) {
      vehicleModel_B->Product[ret] = 0.0;
      vehicleModel_B->Product[ret] += rtb_Transpose1[ret] *
        vehicleModel_B->VectorConcatenate_k[0];
      vehicleModel_B->Product[ret] += rtb_Transpose1[ret + 3] *
        vehicleModel_B->VectorConcatenate_k[1];
      vehicleModel_B->Product[ret] += rtb_Transpose1[ret + 6] *
        vehicleModel_B->VectorConcatenate_k[2];
    }

    /* End of Product: '<S22>/Product' */

    /* Sum: '<S23>/Sum' incorporates:
     *  Product: '<S25>/i x j'
     *  Product: '<S25>/j x k'
     *  Product: '<S25>/k x i'
     *  Product: '<S26>/i x k'
     *  Product: '<S26>/j x i'
     *  Product: '<S26>/k x j'
     */
    vehicleModel_B->Sum[0] = 0.0 * vehicleModel_B->VectorConcatenate_k[2] - 0.0 *
      vehicleModel_B->VectorConcatenate_k[1];
    vehicleModel_B->Sum[1] = 0.0 * vehicleModel_B->VectorConcatenate_k[0] - 0.0 *
      vehicleModel_B->VectorConcatenate_k[2];
    vehicleModel_B->Sum[2] = 0.0 * vehicleModel_B->VectorConcatenate_k[1] - 0.0 *
      vehicleModel_B->VectorConcatenate_k[0];
  }

  /* Trigonometry: '<S35>/sincos' incorporates:
   *  Constant: '<S34>/Constant'
   */
  rtb_sincos_o1_idx_0 = cos(vehicleModel_P.Constant_Value_m);
  rtb_sincos_o1_idx_1 = sin(vehicleModel_P.Constant_Value_m);

  /* Fcn: '<S35>/Fcn11' incorporates:
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_VectorConcatenate[0] = rtb_Product8_idx_1 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S35>/Fcn21' incorporates:
   *  Fcn: '<S35>/Fcn22'
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_sincos_o1_idx_2 = rtb_sincos_o1_idx_1 * rtb_Product8_idx_0;
  rtb_VectorConcatenate[1] = rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_0 -
    rtb_sincos_o1_idx_0 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S35>/Fcn31' incorporates:
   *  Fcn: '<S35>/Fcn32'
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_VectorConcatenate_tmp = rtb_sincos_o1_idx_0 * rtb_Product8_idx_0;
  rtb_VectorConcatenate[2] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_idx_0 +
    rtb_sincos_o1_idx_1 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S35>/Fcn12' incorporates:
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_VectorConcatenate[3] = rtb_Product8_idx_1 * rtb_sincos_o1_idx_1;

  /* Fcn: '<S35>/Fcn22' incorporates:
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_VectorConcatenate[4] = rtb_sincos_o1_idx_2 * rtb_sincos_o1_idx_1 +
    rtb_sincos_o1_idx_0 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S35>/Fcn32' incorporates:
   *  Trigonometry: '<S35>/sincos'
   */
  rtb_VectorConcatenate[5] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_idx_1 -
    rtb_sincos_o1_idx_1 * rtb_sincos_o1_idx_0;

  /* Fcn: '<S35>/Fcn13' */
  rtb_VectorConcatenate[6] = -rtb_Product8_idx_0;

  /* Fcn: '<S35>/Fcn23' */
  rtb_VectorConcatenate[7] = rtb_sincos_o1_idx_1 * rtb_Product8_idx_1;

  /* Fcn: '<S35>/Fcn33' */
  rtb_VectorConcatenate[8] = rtb_sincos_o1_idx_0 * rtb_Product8_idx_1;

  /* Product: '<S34>/Product' incorporates:
   *  Constant: '<S34>/Constant'
   *  Math: '<S34>/Transpose'
   *  SignalConversion generated from: '<S34>/Product'
   */
  for (ret = 0; ret < 3; ret++) {
    rtb_VectorConcatenate5[ret] = rtb_VectorConcatenate[3 * ret + 2] *
      vehicleModel_B->VectorConcatenate2[2] + (rtb_VectorConcatenate[3 * ret + 1]
      * vehicleModel_P.Constant_Value_m + rtb_VectorConcatenate[3 * ret] *
      vehicleModel_B->VectorConcatenate2[0]);
  }

  /* End of Product: '<S34>/Product' */
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    for (ret = 0; ret < 3; ret++) {
      /* Product: '<S21>/Product' */
      vehicleModel_B->Product_j[ret] = 0.0;
      vehicleModel_B->Product_j[ret] += rtb_Transpose1[ret] *
        vehicleModel_B->Sum[0];
      vehicleModel_B->Product_j[ret] += rtb_Transpose1[ret + 3] *
        vehicleModel_B->Sum[1];
      vehicleModel_B->Product_j[ret] += rtb_Transpose1[ret + 6] *
        vehicleModel_B->Sum[2];

      /* Trigonometry: '<S27>/sincos' */
      rtb_sincos_o2[ret] = cos(rtb_sincos_o1_f[ret]);
      rtb_sincos_o1_f[ret] = sin(rtb_sincos_o1_f[ret]);
    }

    /* Fcn: '<S27>/Fcn11' */
    rtb_VectorConcatenate_m[0] = rtb_sincos_o2[1] * rtb_sincos_o2[0];

    /* Fcn: '<S27>/Fcn21' incorporates:
     *  Fcn: '<S27>/Fcn22'
     */
    rtb_VectorConcatenate_tmp = rtb_sincos_o1_f[2] * rtb_sincos_o1_f[1];
    rtb_VectorConcatenate_m[1] = rtb_VectorConcatenate_tmp * rtb_sincos_o2[0] -
      rtb_sincos_o2[2] * rtb_sincos_o1_f[0];

    /* Fcn: '<S27>/Fcn31' incorporates:
     *  Fcn: '<S27>/Fcn32'
     */
    rtb_VectorConcatenate_i_tmp = rtb_sincos_o2[2] * rtb_sincos_o1_f[1];
    rtb_VectorConcatenate_m[2] = rtb_VectorConcatenate_i_tmp * rtb_sincos_o2[0]
      + rtb_sincos_o1_f[2] * rtb_sincos_o1_f[0];

    /* Fcn: '<S27>/Fcn12' */
    rtb_VectorConcatenate_m[3] = rtb_sincos_o2[1] * rtb_sincos_o1_f[0];

    /* Fcn: '<S27>/Fcn22' */
    rtb_VectorConcatenate_m[4] = rtb_VectorConcatenate_tmp * rtb_sincos_o1_f[0]
      + rtb_sincos_o2[2] * rtb_sincos_o2[0];

    /* Fcn: '<S27>/Fcn32' */
    rtb_VectorConcatenate_m[5] = rtb_VectorConcatenate_i_tmp * rtb_sincos_o1_f[0]
      - rtb_sincos_o1_f[2] * rtb_sincos_o2[0];

    /* Fcn: '<S27>/Fcn13' */
    rtb_VectorConcatenate_m[6] = -rtb_sincos_o1_f[1];

    /* Fcn: '<S27>/Fcn23' */
    rtb_VectorConcatenate_m[7] = rtb_sincos_o1_f[2] * rtb_sincos_o2[1];

    /* Fcn: '<S27>/Fcn33' */
    rtb_VectorConcatenate_m[8] = rtb_sincos_o2[2] * rtb_sincos_o2[1];

    /* Math: '<S19>/Transpose1' */
    for (ret = 0; ret < 3; ret++) {
      rtb_Transpose1[3 * ret] = rtb_VectorConcatenate_m[ret];
      rtb_Transpose1[3 * ret + 1] = rtb_VectorConcatenate_m[ret + 3];
      rtb_Transpose1[3 * ret + 2] = rtb_VectorConcatenate_m[ret + 6];
    }

    /* End of Math: '<S19>/Transpose1' */

    /* Constant: '<S19>/R_T1' */
    vehicleModel_B->VectorConcatenate_i[0] =
      -vehicleModel_P.VehicleBody1DOFLongitudinal_b;

    /* Constant: '<S19>/R_T2' */
    vehicleModel_B->VectorConcatenate_i[1] =
      vehicleModel_P.HardPointCoordinateTransformRea;

    /* Constant: '<S19>/R_T3' */
    vehicleModel_B->VectorConcatenate_i[2] =
      vehicleModel_P.VehicleBody1DOFLongitudinal_h;

    /* Sum: '<S30>/Sum' incorporates:
     *  Product: '<S32>/i x j'
     *  Product: '<S32>/j x k'
     *  Product: '<S32>/k x i'
     *  Product: '<S33>/i x k'
     *  Product: '<S33>/j x i'
     *  Product: '<S33>/k x j'
     */
    vehicleModel_B->Sum_m[0] = 0.0 * vehicleModel_B->VectorConcatenate_i[2] -
      0.0 * vehicleModel_B->VectorConcatenate_i[1];
    vehicleModel_B->Sum_m[1] = 0.0 * vehicleModel_B->VectorConcatenate_i[0] -
      0.0 * vehicleModel_B->VectorConcatenate_i[2];
    vehicleModel_B->Sum_m[2] = 0.0 * vehicleModel_B->VectorConcatenate_i[1] -
      0.0 * vehicleModel_B->VectorConcatenate_i[0];
    for (ret = 0; ret < 3; ret++) {
      /* Product: '<S29>/Product' */
      vehicleModel_B->Product_o[ret] = 0.0;

      /* Product: '<S28>/Product' */
      vehicleModel_B->Product_n[ret] = 0.0;

      /* Product: '<S29>/Product' */
      vehicleModel_B->Product_o[ret] += rtb_Transpose1[ret] *
        vehicleModel_B->VectorConcatenate_i[0];

      /* Product: '<S28>/Product' incorporates:
       *  Product: '<S29>/Product'
       */
      vehicleModel_B->Product_n[ret] += rtb_Transpose1[ret] *
        vehicleModel_B->Sum_m[0];

      /* Product: '<S29>/Product' */
      rtb_Product8_idx_0 = rtb_Transpose1[ret + 3];
      vehicleModel_B->Product_o[ret] += rtb_Product8_idx_0 *
        vehicleModel_B->VectorConcatenate_i[1];

      /* Product: '<S28>/Product' */
      vehicleModel_B->Product_n[ret] += rtb_Product8_idx_0 *
        vehicleModel_B->Sum_m[1];

      /* Product: '<S29>/Product' */
      rtb_Product8_idx_0 = rtb_Transpose1[ret + 6];
      vehicleModel_B->Product_o[ret] += rtb_Product8_idx_0 *
        vehicleModel_B->VectorConcatenate_i[2];

      /* Product: '<S28>/Product' */
      vehicleModel_B->Product_n[ret] += rtb_Product8_idx_0 *
        vehicleModel_B->Sum_m[2];
    }
  }

  /* Product: '<S37>/Product8' incorporates:
   *  Constant: '<S3>/FExtConstant'
   */
  /* Unit Conversion - from: gn to: m/s^2
     Expression: output = (9.80665*input) + (0) */
  rtb_Product8_idx_0 = vehicleModel_P.FExtConstant_Value[0] *
    vehicleModel_B->VectorConcatenate2[0];
  rtb_Product8_idx_1 = rtb_TmpSignalConversionAtProd_0 *
    vehicleModel_B->VectorConcatenate2[0];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */

  /* Outport: '<Root>/Info' incorporates:
   *  Constant: '<S12>/Constant1'
   *  Constant: '<S12>/Constant10'
   *  Constant: '<S12>/Constant12'
   *  Constant: '<S12>/Constant2'
   *  Constant: '<S15>/Constant1'
   *  Constant: '<S15>/Constant11'
   *  Constant: '<S15>/Constant8'
   *  Constant: '<S16>/Constant10'
   *  Constant: '<S16>/Constant14'
   *  Constant: '<S16>/Constant3'
   *  Constant: '<S16>/Constant4'
   *  Constant: '<S16>/Constant6'
   *  Constant: '<S16>/Constant8'
   *  Constant: '<S16>/Constant9'
   *  Constant: '<S3>/FExtConstant'
   *  Constant: '<S3>/MExtConstant'
   *  Inport: '<Root>/FwF'
   *  Inport: '<Root>/FwR'
   *  Integrator: '<S15>/Integrator3'
   *  Selector: '<S16>/Selector1'
   *  SignalConversion generated from: '<S4>/Vector Concatenate1'
   *  SignalConversion generated from: '<S7>/Vector Concatenate1'
   *  Sum: '<S18>/Add'
   *  Sum: '<S18>/Add1'
   *  Sum: '<S18>/Add4'
   *  Sum: '<S19>/Add'
   *  Sum: '<S19>/Add4'
   *  UnaryMinus: '<S16>/Unary Minus'
   *  UnaryMinus: '<S16>/Unary Minus1'
   * */
  vehicleModel_Y->Info[0] = rtb_VectorConcatenate6_idx_0;
  vehicleModel_Y->Info[1] = vehicleModel_P.Constant12_Value;
  vehicleModel_Y->Info[2] = rtb_VectorConcatenate6_idx_2;
  vehicleModel_Y->Info[3] = rtb_VectorConcatenate5[0];
  vehicleModel_Y->Info[4] = vehicleModel_P.Constant10_Value_o;
  vehicleModel_Y->Info[5] = rtb_VectorConcatenate5[2];
  vehicleModel_Y->Info[6] = vehicleModel_P.Constant1_Value_l;
  vehicleModel_Y->Info[7] = 0.0;
  vehicleModel_Y->Info[8] = vehicleModel_P.Constant2_Value;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[9] = rtb_VectorConcatenate6_idx_0 +
    vehicleModel_B->Product[0];
  vehicleModel_Y->Info[10] = vehicleModel_P.Constant12_Value +
    vehicleModel_B->Product[1];
  vehicleModel_Y->Info[11] = rtb_VectorConcatenate6_idx_2 +
    vehicleModel_B->Product[2];
  vehicleModel_Y->Info[12] = rtb_VectorConcatenate5[0] +
    vehicleModel_B->Product_j[0];
  vehicleModel_Y->Info[13] = vehicleModel_P.Constant10_Value_o +
    vehicleModel_B->Product_j[1];
  vehicleModel_Y->Info[14] = rtb_VectorConcatenate5[2] +
    vehicleModel_B->Product_j[2];
  vehicleModel_Y->Info[15] = rtb_VectorConcatenate6_idx_0 +
    vehicleModel_B->Product_o[0];
  vehicleModel_Y->Info[16] = vehicleModel_P.Constant12_Value +
    vehicleModel_B->Product_o[1];
  vehicleModel_Y->Info[17] = rtb_VectorConcatenate6_idx_2 +
    vehicleModel_B->Product_o[2];
  vehicleModel_Y->Info[18] = rtb_VectorConcatenate5[0] +
    vehicleModel_B->Product_n[0];
  vehicleModel_Y->Info[19] = vehicleModel_P.Constant10_Value_o +
    vehicleModel_B->Product_n[1];
  vehicleModel_Y->Info[20] = rtb_VectorConcatenate5[2] +
    vehicleModel_B->Product_n[2];
  vehicleModel_Y->Info[21] = vehicleModel_X->Integrator3_CSTATE;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[22] = vehicleModel_P.Constant11_Value;
  vehicleModel_Y->Info[23] = vehicleModel_P.Constant11_Value;
  vehicleModel_Y->Info[30] = rtb_Product2;
  vehicleModel_Y->Info[31] = vehicleModel_P.Constant8_Value;
  vehicleModel_Y->Info[32] = vehicleModel_P.Constant1_Value;
  vehicleModel_Y->Info[42] = vehicleModel_U->FwF;
  vehicleModel_Y->Info[43] = vehicleModel_P.Constant3_Value;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[44] = -rtb_VectorConcatenate4_idx_2;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[45] = vehicleModel_U->FwR;
  vehicleModel_Y->Info[46] = vehicleModel_P.Constant4_Value_h;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[47] = -rtb_Product_b;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[48] = vehicleModel_P.Constant6_Value;
  vehicleModel_Y->Info[49] = vehicleModel_P.Constant8_Value_p;
  vehicleModel_Y->Info[50] = rtb_Add6;
  vehicleModel_Y->Info[51] = vehicleModel_P.Constant9_Value;
  vehicleModel_Y->Info[52] = vehicleModel_P.Constant10_Value;
  vehicleModel_Y->Info[53] = rtb_Add_e;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[57] = rtb_Product1_f;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[58] = vehicleModel_P.Constant14_Value;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[59] = rtb_Product2_h;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[24] = vehicleModel_B->VectorConcatenate2[0];
  vehicleModel_Y->Info[27] = 0.0;
  vehicleModel_Y->Info[33] = 0.0;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[36] = rtb_Add_n;

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[39] = vehicleModel_P.FExtConstant_Value[0];
  vehicleModel_Y->Info[54] = rtb_TmpSignalConversionAtProd_0;
  vehicleModel_Y->Info[60] = 0.0;
  vehicleModel_Y->Info[66] = vehicleModel_P.MExtConstant_Value[0];
  vehicleModel_Y->Info[69] = vehicleModel_B->VectorConcatenate_k[0];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[72] = vehicleModel_B->Sum[0] +
    vehicleModel_B->VectorConcatenate2[0];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[25] = vehicleModel_B->VectorConcatenate2[1];
  vehicleModel_Y->Info[28] = 0.0;
  vehicleModel_Y->Info[34] = 0.0;
  vehicleModel_Y->Info[37] = 0.0;
  vehicleModel_Y->Info[40] = vehicleModel_P.FExtConstant_Value[1];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Switch: '<S44>/Switch' incorporates:
   *  Constant: '<S44>/Constant4'
   */
  if (0.0 >= vehicleModel_P.Switch_Threshold) {
    rtb_Add_n = vehicleModel_P.Constant4_Value[1];
  } else {
    rtb_Add_n = vehicleModel_B->UnaryMinus[1];
  }

  /* Outport: '<Root>/Info' incorporates:
   *  Constant: '<S37>/Constant'
   *  Constant: '<S37>/Constant1'
   *  Constant: '<S3>/FExtConstant'
   *  Constant: '<S3>/MExtConstant'
   *  Inport: '<Root>/FwF'
   *  Inport: '<Root>/FwR'
   *  Product: '<S37>/Product'
   *  Product: '<S37>/Product1'
   *  Product: '<S37>/Product8'
   *  Product: '<S44>/Product3'
   *  Sum: '<S18>/Add1'
   *  Sum: '<S19>/Add1'
   *  UnaryMinus: '<S37>/Unary Minus'
   *  UnaryMinus: '<S37>/Unary Minus1'
   *  UnitConversion: '<S37>/Unit Conversion'
   */
  vehicleModel_Y->Info[55] = rtb_Add_n * rtb_uAPabsRT[1];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[61] = 0.0;
  vehicleModel_Y->Info[64] = rtb_Product4_idx_1;
  vehicleModel_Y->Info[67] = vehicleModel_P.MExtConstant_Value[1];
  vehicleModel_Y->Info[70] = vehicleModel_B->VectorConcatenate_k[1];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[73] = vehicleModel_B->Sum[1] +
    vehicleModel_B->VectorConcatenate2[1];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[26] = vehicleModel_B->VectorConcatenate2[2];
  vehicleModel_Y->Info[29] = 0.0;
  vehicleModel_Y->Info[35] = 0.0;
  vehicleModel_Y->Info[38] = 0.0;
  vehicleModel_Y->Info[41] = vehicleModel_P.FExtConstant_Value[2];
  vehicleModel_Y->Info[56] = rtb_VectorConcatenate5_b_idx_2;
  vehicleModel_Y->Info[62] = 0.0;
  vehicleModel_Y->Info[68] = vehicleModel_P.MExtConstant_Value[2];
  vehicleModel_Y->Info[71] = vehicleModel_B->VectorConcatenate_k[2];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[74] = vehicleModel_B->Sum[2] +
    vehicleModel_B->VectorConcatenate2[2];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[75] = 0.0;
  vehicleModel_Y->Info[76] = 0.0;
  vehicleModel_Y->Info[77] = vehicleModel_B->VectorConcatenate_i[0];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[80] = vehicleModel_B->Sum_m[0] +
    vehicleModel_B->VectorConcatenate2[0];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[78] = vehicleModel_B->VectorConcatenate_i[1];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[81] = vehicleModel_B->Sum_m[1] +
    vehicleModel_B->VectorConcatenate2[1];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[79] = vehicleModel_B->VectorConcatenate_i[2];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[82] = vehicleModel_B->Sum_m[2] +
    vehicleModel_B->VectorConcatenate2[2];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[87] = rtb_Product8_idx_0;
  vehicleModel_Y->Info[83] = 0.0;
  vehicleModel_Y->Info[85] = rtb_Product8_idx_0;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[88] = vehicleModel_U->FwF *
    vehicleModel_B->VectorConcatenate2[0];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[84] = 0.0;
  vehicleModel_Y->Info[86] = rtb_Product8_idx_1;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  vehicleModel_Y->Info[89] = vehicleModel_U->FwR *
    vehicleModel_B->VectorConcatenate2[0];
  vehicleModel_Y->Info[90] = -rtb_Product8_idx_1;
  vehicleModel_Y->Info[91] = -(vehicleModel_P.VehicleBody1DOFLongitudinal_m *
    vehicleModel_P.VehicleBody1DOFLongitudinal_g * rtb_VectorConcatenate5[2]);
  vehicleModel_Y->Info[92] = vehicleModel_P.VehicleBody1DOFLongitudinal_m *
    vehicleModel_B->VectorConcatenate2[0] * (9.8066500000000012 * rtb_Product2);

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */

  /* Outport: '<Root>/FzF' */
  vehicleModel_Y->FzF = rtb_Add6;

  /* Outport: '<Root>/FzR' */
  vehicleModel_Y->FzR = rtb_Add_e;

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Outport: '<Root>/state' incorporates:
   *  Integrator: '<S2>/Integrator'
   */
  vehicleModel_Y->state[0] = vehicleModel_X->Integrator_CSTATE[0];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */

  /* Outport: '<Root>/stateDot' */
  vehicleModel_Y->stateDot[0] = vehicleModel_B->MATLABSystem[0];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Outport: '<Root>/state' incorporates:
   *  Integrator: '<S2>/Integrator'
   */
  vehicleModel_Y->state[1] = vehicleModel_X->Integrator_CSTATE[1];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */

  /* Outport: '<Root>/stateDot' */
  vehicleModel_Y->stateDot[1] = vehicleModel_B->MATLABSystem[1];

  /* Outputs for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Outport: '<Root>/state' incorporates:
   *  Integrator: '<S2>/Integrator'
   */
  vehicleModel_Y->state[2] = vehicleModel_X->Integrator_CSTATE[2];

  /* End of Outputs for SubSystem: '<Root>/Atomic Subsystem' */

  /* Outport: '<Root>/stateDot' */
  vehicleModel_Y->stateDot[2] = vehicleModel_B->MATLABSystem[2];
  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* Matfile logging */
    // Commented out by REZA:
    // rt_UpdateTXYLogVars(vehicleModel_M->rtwLogInfo, (vehicleModel_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(vehicleModel_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(vehicleModel_M)!=-1) &&
          !((rtmGetTFinal(vehicleModel_M)-(((vehicleModel_M->Timing.clockTick1+
               vehicleModel_M->Timing.clockTickH1* 4294967296.0)) * 0.01)) >
            (((vehicleModel_M->Timing.clockTick1+
               vehicleModel_M->Timing.clockTickH1* 4294967296.0)) * 0.01) *
            (DBL_EPSILON))) {
        rtmSetErrorStatus(vehicleModel_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(vehicleModel_M->solverInfo, vehicleModel_M);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++vehicleModel_M->Timing.clockTick0)) {
      ++vehicleModel_M->Timing.clockTickH0;
    }

    vehicleModel_M->Timing.t[0] = rtsiGetSolverStopTime
      (vehicleModel_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      vehicleModel_M->Timing.clockTick1++;
      if (!vehicleModel_M->Timing.clockTick1) {
        vehicleModel_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void vehicleModel_derivatives(RT_MODEL_vehicleModel_T *const vehicleModel_M)
{
  B_vehicleModel_T *vehicleModel_B = ((B_vehicleModel_T *)
    vehicleModel_M->blockIO);
  ExtU_vehicleModel_T *vehicleModel_U = (ExtU_vehicleModel_T *)
    vehicleModel_M->inputs;
  XDot_vehicleModel_T *_rtXdot;
  _rtXdot = ((XDot_vehicleModel_T *) vehicleModel_M->derivs);

  /* Derivatives for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* Derivatives for Integrator: '<S2>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = vehicleModel_B->MATLABSystem[0];
  _rtXdot->Integrator_CSTATE[1] = vehicleModel_B->MATLABSystem[1];
  _rtXdot->Integrator_CSTATE[2] = vehicleModel_B->MATLABSystem[2];

  /* Derivatives for Integrator: '<S7>/Integrator' */
  _rtXdot->Integrator_CSTATE_b = vehicleModel_B->xddot;

  /* Derivatives for Integrator: '<S1>/Integrator' incorporates:
   *  Inport: '<Root>/steeringRate'
   */
  _rtXdot->Integrator_CSTATE_l = vehicleModel_U->steeringRate;

  /* Derivatives for Integrator: '<S15>/Integrator3' */
  _rtXdot->Integrator3_CSTATE = vehicleModel_B->VectorConcatenate2[0];

  /* Derivatives for Integrator: '<S4>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = vehicleModel_B->Integrator;

  /* End of Derivatives for SubSystem: '<Root>/Atomic Subsystem' */
}

/* Model initialize function */
void vehicleModel_initialize(RT_MODEL_vehicleModel_T *const vehicleModel_M)
{
  DW_vehicleModel_T *vehicleModel_DW = ((DW_vehicleModel_T *)
    vehicleModel_M->dwork);
  X_vehicleModel_T *vehicleModel_X = ((X_vehicleModel_T *)
    vehicleModel_M->contStates);

  {
    boolean_T flag;

    /* Start for Atomic SubSystem: '<Root>/Atomic Subsystem' */
    /* Start for MATLABSystem: '<S2>/MATLAB System' */
    vehicleModel_DW->obj.isInitialized = 0;
    vehicleModel_DW->obj.tunablePropertyChanged[0] = false;
    vehicleModel_DW->obj.tunablePropertyChanged[1] = false;
    vehicleModel_DW->obj.tunablePropertyChanged[2] = false;
    vehicleModel_DW->objisempty = true;
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[0] = true;
    }

    vehicleModel_DW->obj.WheelBase =
      vehicleModel_P.BicycleKinematicModel_WheelBase;
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[1] = true;
    }

    vehicleModel_DW->obj.VehicleSpeedRange[0] =
      vehicleModel_P.BicycleKinematicModel_VehicleSp[0];
    vehicleModel_DW->obj.VehicleSpeedRange[1] =
      vehicleModel_P.BicycleKinematicModel_VehicleSp[1];
    flag = (vehicleModel_DW->obj.isInitialized == 1);
    if (flag) {
      vehicleModel_DW->obj.TunablePropsChanged = true;
      vehicleModel_DW->obj.tunablePropertyChanged[2] = true;
    }

    vehicleModel_DW->obj.MaxSteeringAngle =
      vehicleModel_P.BicycleKinematicModel_MaxSteeri;
    vehicleModel_SystemCore_setup(&vehicleModel_DW->obj);

    /* End of Start for MATLABSystem: '<S2>/MATLAB System' */
    /* End of Start for SubSystem: '<Root>/Atomic Subsystem' */
  }

  /* SystemInitialize for Atomic SubSystem: '<Root>/Atomic Subsystem' */
  /* InitializeConditions for Integrator: '<S2>/Integrator' */
  vehicleModel_X->Integrator_CSTATE[0] =
    vehicleModel_P.BicycleKinematicModel_InitialSt[0];
  vehicleModel_X->Integrator_CSTATE[1] =
    vehicleModel_P.BicycleKinematicModel_InitialSt[1];
  vehicleModel_X->Integrator_CSTATE[2] =
    vehicleModel_P.BicycleKinematicModel_InitialSt[2];

  /* InitializeConditions for Integrator: '<S7>/Integrator' */
  vehicleModel_X->Integrator_CSTATE_b =
    vehicleModel_P.VehicleBody1DOFLongitudinal_xdo;

  /* InitializeConditions for Integrator: '<S1>/Integrator' */
  vehicleModel_X->Integrator_CSTATE_l = vehicleModel_P.Integrator_IC;

  /* InitializeConditions for Integrator: '<S15>/Integrator3' */
  vehicleModel_X->Integrator3_CSTATE =
    vehicleModel_P.VehicleBody1DOFLongitudinal_xdo;

  /* InitializeConditions for Integrator: '<S4>/Integrator1' */
  vehicleModel_X->Integrator1_CSTATE =
    vehicleModel_P.VehicleBody1DOFLongitudinal_x_o;

  /* InitializeConditions for MATLABSystem: '<S2>/MATLAB System' */
  BicycleKinematics_assignModelPr(&vehicleModel_DW->obj);

  /* End of SystemInitialize for SubSystem: '<Root>/Atomic Subsystem' */
}

/* Model terminate function */
void vehicleModel_terminate(RT_MODEL_vehicleModel_T * vehicleModel_M)
{
  rt_FREE(vehicleModel_M->solverInfo);

  /* model code */
  rt_FREE(vehicleModel_M->blockIO);
  rt_FREE(vehicleModel_M->contStates);
  rt_FREE(vehicleModel_M->inputs);
  rt_FREE(vehicleModel_M->outputs);
  rt_FREE(vehicleModel_M->dwork);

  {
    void *xptr = (void *) rtliGetLogXSignalPtrs(vehicleModel_M->rtwLogInfo);
    void *yptr = (void *) rtliGetLogYSignalPtrs(vehicleModel_M->rtwLogInfo);
    rt_FREE(xptr);
    rt_FREE(yptr);
  }

  rt_FREE(vehicleModel_M->rtwLogInfo);
  rt_FREE(vehicleModel_M);
}

/* Model data allocation function */
RT_MODEL_vehicleModel_T *vehicleModel(void)
{
  RT_MODEL_vehicleModel_T *vehicleModel_M;
  vehicleModel_M = (RT_MODEL_vehicleModel_T *) malloc(sizeof
    (RT_MODEL_vehicleModel_T));
  if (vehicleModel_M == NULL) {
    return NULL;
  }

  (void) memset((char *)vehicleModel_M, 0,
                sizeof(RT_MODEL_vehicleModel_T));

  {
    /* Setup solver object */
    RTWSolverInfo *rt_SolverInfo = (RTWSolverInfo *) malloc(sizeof(RTWSolverInfo));
    rt_VALIDATE_MEMORY(vehicleModel_M,rt_SolverInfo);
    vehicleModel_M->solverInfo = (rt_SolverInfo);
    rtsiSetSimTimeStepPtr(vehicleModel_M->solverInfo,
                          &vehicleModel_M->Timing.simTimeStep);
    rtsiSetTPtr(vehicleModel_M->solverInfo, &rtmGetTPtr(vehicleModel_M));
    rtsiSetStepSizePtr(vehicleModel_M->solverInfo,
                       &vehicleModel_M->Timing.stepSize0);
    rtsiSetdXPtr(vehicleModel_M->solverInfo, &vehicleModel_M->derivs);
    rtsiSetContStatesPtr(vehicleModel_M->solverInfo, (real_T **)
                         &vehicleModel_M->contStates);
    rtsiSetNumContStatesPtr(vehicleModel_M->solverInfo,
      &vehicleModel_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(vehicleModel_M->solverInfo,
      &vehicleModel_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(vehicleModel_M->solverInfo,
      &vehicleModel_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(vehicleModel_M->solverInfo,
      &vehicleModel_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(vehicleModel_M->solverInfo, (&rtmGetErrorStatus
      (vehicleModel_M)));
    rtsiSetRTModelPtr(vehicleModel_M->solverInfo, vehicleModel_M);
  }

  rtsiSetSolverName(vehicleModel_M->solverInfo,"ode3");

  /* Setup for data logging */
  {
    RTWLogInfo *rt_DataLoggingInfo = (RTWLogInfo *) malloc(sizeof(RTWLogInfo));
    rt_VALIDATE_MEMORY(vehicleModel_M,rt_DataLoggingInfo);
    rt_DataLoggingInfo->loggingInterval = NULL;
    vehicleModel_M->rtwLogInfo = rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(vehicleModel_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(vehicleModel_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalInfo(vehicleModel_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(vehicleModel_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  {
    B_vehicleModel_T *b = (B_vehicleModel_T *) malloc(sizeof(B_vehicleModel_T));
    rt_VALIDATE_MEMORY(vehicleModel_M,b);
    vehicleModel_M->blockIO = (b);
  }

  /* states (continuous) */
  {
    X_vehicleModel_T *x = (X_vehicleModel_T *) malloc(sizeof(X_vehicleModel_T));
    rt_VALIDATE_MEMORY(vehicleModel_M,x);
    vehicleModel_M->contStates = (x);
  }

  /* states (dwork) */
  {
    DW_vehicleModel_T *dwork = (DW_vehicleModel_T *) malloc(sizeof
      (DW_vehicleModel_T));
    rt_VALIDATE_MEMORY(vehicleModel_M,dwork);
    vehicleModel_M->dwork = (dwork);
  }

  /* external inputs */
  {
    ExtU_vehicleModel_T *vehicleModel_U = (ExtU_vehicleModel_T *) malloc(sizeof
      (ExtU_vehicleModel_T));
    rt_VALIDATE_MEMORY(vehicleModel_M,vehicleModel_U);
    vehicleModel_M->inputs = (((ExtU_vehicleModel_T *) vehicleModel_U));
  }

  /* external outputs */
  {
    ExtY_vehicleModel_T *vehicleModel_Y = (ExtY_vehicleModel_T *) malloc(sizeof
      (ExtY_vehicleModel_T));
    rt_VALIDATE_MEMORY(vehicleModel_M,vehicleModel_Y);
    vehicleModel_M->outputs = (vehicleModel_Y);
  }

  {
    B_vehicleModel_T *vehicleModel_B = ((B_vehicleModel_T *)
      vehicleModel_M->blockIO);
    DW_vehicleModel_T *vehicleModel_DW = ((DW_vehicleModel_T *)
      vehicleModel_M->dwork);
    X_vehicleModel_T *vehicleModel_X = ((X_vehicleModel_T *)
      vehicleModel_M->contStates);
    ExtU_vehicleModel_T *vehicleModel_U = (ExtU_vehicleModel_T *)
      vehicleModel_M->inputs;
    ExtY_vehicleModel_T *vehicleModel_Y = (ExtY_vehicleModel_T *)
      vehicleModel_M->outputs;

    /* initialize non-finites */
    rt_InitInfAndNaN(sizeof(real_T));

    /* non-finite (run-time) assignments */
    vehicleModel_P.BicycleKinematicModel_VehicleSp[0] = rtMinusInf;
    vehicleModel_P.BicycleKinematicModel_VehicleSp[1] = rtInf;
    rtsiSetSimTimeStep(vehicleModel_M->solverInfo, MAJOR_TIME_STEP);
    vehicleModel_M->intgData.y = vehicleModel_M->odeY;
    vehicleModel_M->intgData.f[0] = vehicleModel_M->odeF[0];
    vehicleModel_M->intgData.f[1] = vehicleModel_M->odeF[1];
    vehicleModel_M->intgData.f[2] = vehicleModel_M->odeF[2];
    vehicleModel_M->contStates = ((X_vehicleModel_T *) vehicleModel_X);
    rtsiSetSolverData(vehicleModel_M->solverInfo, (void *)
                      &vehicleModel_M->intgData);
    rtmSetTPtr(vehicleModel_M, &vehicleModel_M->Timing.tArray[0]);
    rtmSetTFinal(vehicleModel_M, 10.0);
    vehicleModel_M->Timing.stepSize0 = 0.01;

    /* Setup for data logging */
    {
      rtliSetLogT(vehicleModel_M->rtwLogInfo, "tout");
      rtliSetLogX(vehicleModel_M->rtwLogInfo, "");
      rtliSetLogXFinal(vehicleModel_M->rtwLogInfo, "");
      rtliSetLogVarNameModifier(vehicleModel_M->rtwLogInfo, "rt_");
      rtliSetLogFormat(vehicleModel_M->rtwLogInfo, 4);
      rtliSetLogMaxRows(vehicleModel_M->rtwLogInfo, 0);
      rtliSetLogDecimation(vehicleModel_M->rtwLogInfo, 1);
      rtliSetLogY(vehicleModel_M->rtwLogInfo, "");
    }

    vehicleModel_M->Timing.stepSize = (0.01);

    /* block I/O */
    (void) memset(((void *) vehicleModel_B), 0,
                  sizeof(B_vehicleModel_T));

    /* states (continuous) */
    {
      (void) memset((void *)vehicleModel_X, 0,
                    sizeof(X_vehicleModel_T));
    }

    /* states (dwork) */
    (void) memset((void *)vehicleModel_DW, 0,
                  sizeof(DW_vehicleModel_T));

    /* external inputs */
    (void)memset(vehicleModel_U, 0, sizeof(ExtU_vehicleModel_T));

    /* external outputs */
    (void) memset((void *)vehicleModel_Y, 0,
                  sizeof(ExtY_vehicleModel_T));
  }

  return vehicleModel_M;
}
