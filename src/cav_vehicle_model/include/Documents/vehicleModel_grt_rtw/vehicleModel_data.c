/*
 * vehicleModel_data.c
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

/* Block parameters (default storage) */
P_vehicleModel_T vehicleModel_P = {
  /* Mask Parameter: VehicleBody1DOFLongitudinal_Af
   * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
   */
  4.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_Cd
   * Referenced by: '<S44>/Constant'
   */
  0.3,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_Cl
   * Referenced by: '<S44>/Constant1'
   */
  0.3,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_Cpm
   * Referenced by: '<S44>/Constant2'
   */
  0.3,

  /* Mask Parameter: DragForce_Cs
   * Referenced by: '<S44>/Cs'
   */
  { 0.0, 0.0 },

  /* Mask Parameter: DragForce_Cym
   * Referenced by: '<S44>/Cym'
   */
  { 0.0, 0.0 },

  /* Mask Parameter: BicycleKinematicModel_InitialSt
   * Referenced by: '<S2>/Integrator'
   */
  { 0.0, 0.0, 0.0 },

  /* Mask Parameter: BicycleKinematicModel_MaxSteeri
   * Referenced by: '<S2>/MATLAB System'
   */
  0.78539816339744828,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_NF
   * Referenced by: '<S16>/1//NF'
   */
  2.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_NR
   * Referenced by: '<S16>/1//NR'
   */
  2.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_Pab
   * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
   */
  101325.0,

  /* Mask Parameter: DragForce_R
   * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
   */
  287.058,

  /* Mask Parameter: HardPointCoordinateTransformFro
   * Referenced by: '<S18>/R_T2'
   */
  0.0,

  /* Mask Parameter: HardPointCoordinateTransformRea
   * Referenced by: '<S19>/R_T2'
   */
  0.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_T
   * Referenced by: '<S3>/AirTempConstant'
   */
  273.0,

  /* Mask Parameter: BicycleKinematicModel_VehicleSp
   * Referenced by: '<S2>/MATLAB System'
   */
  { 0.0, 0.0 },

  /* Mask Parameter: BicycleKinematicModel_WheelBase
   * Referenced by: '<S2>/MATLAB System'
   */
  2.7,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_a
   * Referenced by:
   *   '<S7>/1//(a+b)'
   *   '<S7>/1//(a+b) '
   *   '<S7>/a'
   *   '<S44>/Constant3'
   *   '<S18>/R_T1'
   */
  1.2,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_b
   * Referenced by:
   *   '<S7>/1//(a+b)'
   *   '<S7>/1//(a+b) '
   *   '<S7>/b'
   *   '<S44>/Constant3'
   *   '<S19>/R_T1'
   */
  1.6,

  /* Mask Parameter: DragForce_beta_w
   * Referenced by:
   *   '<S44>/Cs'
   *   '<S44>/Cym'
   */
  { -1.0, 1.0 },

  /* Mask Parameter: VehicleBody1DOFLongitudinal_g
   * Referenced by:
   *   '<S7>/g'
   *   '<S37>/Constant'
   */
  9.81,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_h
   * Referenced by:
   *   '<S7>/h'
   *   '<S18>/R_T3'
   *   '<S19>/R_T3'
   */
  0.35,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_m
   * Referenced by:
   *   '<S7>/1//m'
   *   '<S7>/m'
   *   '<S37>/Constant'
   *   '<S37>/Constant1'
   */
  1200.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_x_o
   * Referenced by: '<S4>/Integrator1'
   */
  0.0,

  /* Mask Parameter: VehicleBody1DOFLongitudinal_xdo
   * Referenced by:
   *   '<S7>/Integrator'
   *   '<S15>/Integrator3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S1>/Integrator'
   */
  0.0,

  /* Expression: [0,0,0]
   * Referenced by: '<S3>/FExtConstant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0,0,0]
   * Referenced by: '<S3>/MExtConstant'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<S15>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S15>/Constant11'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S15>/Constant8'
   */
  0.0,

  /* Expression: ones(1,3)
   * Referenced by: '<S44>/Constant4'
   */
  { 1.0, 1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S44>/Switch'
   */
  0.0,

  /* Expression: [0 0]
   * Referenced by: '<S44>/Crm'
   */
  { 0.0, 0.0 },

  /* Expression: [-1 1]
   * Referenced by: '<S44>/Crm'
   */
  { -1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S16>/Constant10'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant14'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant4'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant6'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant8'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S16>/Constant9'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S6>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S12>/Constant12'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S12>/Constant2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S12>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S34>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S12>/Constant10'
   */
  0.0
};
