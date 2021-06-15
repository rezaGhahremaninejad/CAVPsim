/*
 * vehicleModel.h
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

#ifndef RTW_HEADER_vehicleModel_h_
#define RTW_HEADER_vehicleModel_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef vehicleModel_COMMON_INCLUDES_
# define vehicleModel_COMMON_INCLUDES_
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* vehicleModel_COMMON_INCLUDES_ */

#include "vehicleModel_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetBlockIO
# define rtmGetBlockIO(rtm)            ((rtm)->blockIO)
#endif

#ifndef rtmSetBlockIO
# define rtmSetBlockIO(rtm, val)       ((rtm)->blockIO = (val))
#endif

#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetRootDWork
# define rtmGetRootDWork(rtm)          ((rtm)->dwork)
#endif

#ifndef rtmSetRootDWork
# define rtmSetRootDWork(rtm, val)     ((rtm)->dwork = (val))
#endif

#ifndef rtmGetStepSize
# define rtmGetStepSize(rtm)           ((rtm)->Timing.stepSize)
#endif

#ifndef rtmGetU
# define rtmGetU(rtm)                  ((rtm)->inputs)
#endif

#ifndef rtmSetU
# define rtmSetU(rtm, val)             ((rtm)->inputs = (val))
#endif

#ifndef rtmGetY
# define rtmGetY(rtm)                  ((rtm)->outputs)
#endif

#ifndef rtmSetY
# define rtmSetY(rtm, val)             ((rtm)->outputs = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#define vehicleModel_M_TYPE            RT_MODEL_vehicleModel_T

/* Block signals (default storage) */
typedef struct {
  real_T Integrator;                   /* '<S7>/Integrator' */
  real_T VectorConcatenate2[3];        /* '<S4>/Vector Concatenate2' */
  real_T Fz;                           /* '<S7>/m' */
  real_T UnaryMinus[3];                /* '<S44>/Unary Minus' */
  real_T VectorConcatenate[6];         /* '<S44>/Vector Concatenate' */
  real_T xddot;                        /* '<S7>/1//m' */
  real_T VectorConcatenate_k[3];       /* '<S18>/Vector Concatenate' */
  real_T Product[3];                   /* '<S22>/Product' */
  real_T Sum[3];                       /* '<S23>/Sum' */
  real_T Product_j[3];                 /* '<S21>/Product' */
  real_T VectorConcatenate_i[3];       /* '<S19>/Vector Concatenate' */
  real_T Product_o[3];                 /* '<S29>/Product' */
  real_T Sum_m[3];                     /* '<S30>/Sum' */
  real_T Product_n[3];                 /* '<S28>/Product' */
  real_T MATLABSystem[3];              /* '<S2>/MATLAB System' */
} B_vehicleModel_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slmobile_internal_bl_T obj; /* '<S2>/MATLAB System' */
  boolean_T objisempty;                /* '<S2>/MATLAB System' */
} DW_vehicleModel_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S2>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S7>/Integrator' */
  real_T Integrator_CSTATE_l;          /* '<S1>/Integrator' */
  real_T Integrator3_CSTATE;           /* '<S15>/Integrator3' */
  real_T Integrator1_CSTATE;           /* '<S4>/Integrator1' */
} X_vehicleModel_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S2>/Integrator' */
  real_T Integrator_CSTATE_b;          /* '<S7>/Integrator' */
  real_T Integrator_CSTATE_l;          /* '<S1>/Integrator' */
  real_T Integrator3_CSTATE;           /* '<S15>/Integrator3' */
  real_T Integrator1_CSTATE;           /* '<S4>/Integrator1' */
} XDot_vehicleModel_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[3];      /* '<S2>/Integrator' */
  boolean_T Integrator_CSTATE_b;       /* '<S7>/Integrator' */
  boolean_T Integrator_CSTATE_l;       /* '<S1>/Integrator' */
  boolean_T Integrator3_CSTATE;        /* '<S15>/Integrator3' */
  boolean_T Integrator1_CSTATE;        /* '<S4>/Integrator1' */
} XDis_vehicleModel_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T FwF;                          /* '<Root>/FwF' */
  real_T FwR;                          /* '<Root>/FwR' */
  real_T Grade;                        /* '<Root>/Grade' */
  real_T WindX;                        /* '<Root>/WindX' */
  real_T steeringRate;                 /* '<Root>/steeringRate' */
} ExtU_vehicleModel_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Info[93];                     /* '<Root>/Info' */
  real_T FzF;                          /* '<Root>/FzF' */
  real_T FzR;                          /* '<Root>/FzR' */
  real_T state[3];                     /* '<Root>/state' */
  real_T stateDot[3];                  /* '<Root>/stateDot' */
} ExtY_vehicleModel_T;

/* Parameters (default storage) */
struct P_vehicleModel_T_ {
  real_T VehicleBody1DOFLongitudinal_Af;
                               /* Mask Parameter: VehicleBody1DOFLongitudinal_Af
                                * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
                                */
  real_T VehicleBody1DOFLongitudinal_Cd;
                               /* Mask Parameter: VehicleBody1DOFLongitudinal_Cd
                                * Referenced by: '<S44>/Constant'
                                */
  real_T VehicleBody1DOFLongitudinal_Cl;
                               /* Mask Parameter: VehicleBody1DOFLongitudinal_Cl
                                * Referenced by: '<S44>/Constant1'
                                */
  real_T VehicleBody1DOFLongitudinal_Cpm;
                              /* Mask Parameter: VehicleBody1DOFLongitudinal_Cpm
                               * Referenced by: '<S44>/Constant2'
                               */
  real_T DragForce_Cs[2];              /* Mask Parameter: DragForce_Cs
                                        * Referenced by: '<S44>/Cs'
                                        */
  real_T DragForce_Cym[2];             /* Mask Parameter: DragForce_Cym
                                        * Referenced by: '<S44>/Cym'
                                        */
  real_T BicycleKinematicModel_InitialSt[3];
                              /* Mask Parameter: BicycleKinematicModel_InitialSt
                               * Referenced by: '<S2>/Integrator'
                               */
  real_T BicycleKinematicModel_MaxSteeri;
                              /* Mask Parameter: BicycleKinematicModel_MaxSteeri
                               * Referenced by: '<S2>/MATLAB System'
                               */
  real_T VehicleBody1DOFLongitudinal_NF;
                               /* Mask Parameter: VehicleBody1DOFLongitudinal_NF
                                * Referenced by: '<S16>/1//NF'
                                */
  real_T VehicleBody1DOFLongitudinal_NR;
                               /* Mask Parameter: VehicleBody1DOFLongitudinal_NR
                                * Referenced by: '<S16>/1//NR'
                                */
  real_T VehicleBody1DOFLongitudinal_Pab;
                              /* Mask Parameter: VehicleBody1DOFLongitudinal_Pab
                               * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
                               */
  real_T DragForce_R;                  /* Mask Parameter: DragForce_R
                                        * Referenced by: '<S44>/.5.*A.*Pabs.//R.//T'
                                        */
  real_T HardPointCoordinateTransformFro;
                              /* Mask Parameter: HardPointCoordinateTransformFro
                               * Referenced by: '<S18>/R_T2'
                               */
  real_T HardPointCoordinateTransformRea;
                              /* Mask Parameter: HardPointCoordinateTransformRea
                               * Referenced by: '<S19>/R_T2'
                               */
  real_T VehicleBody1DOFLongitudinal_T;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_T
                                 * Referenced by: '<S3>/AirTempConstant'
                                 */
  real_T BicycleKinematicModel_VehicleSp[2];
                              /* Mask Parameter: BicycleKinematicModel_VehicleSp
                               * Referenced by: '<S2>/MATLAB System'
                               */
  real_T BicycleKinematicModel_WheelBase;
                              /* Mask Parameter: BicycleKinematicModel_WheelBase
                               * Referenced by: '<S2>/MATLAB System'
                               */
  real_T VehicleBody1DOFLongitudinal_a;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_a
                                 * Referenced by:
                                 *   '<S7>/1//(a+b)'
                                 *   '<S7>/1//(a+b) '
                                 *   '<S7>/a'
                                 *   '<S44>/Constant3'
                                 *   '<S18>/R_T1'
                                 */
  real_T VehicleBody1DOFLongitudinal_b;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_b
                                 * Referenced by:
                                 *   '<S7>/1//(a+b)'
                                 *   '<S7>/1//(a+b) '
                                 *   '<S7>/b'
                                 *   '<S44>/Constant3'
                                 *   '<S19>/R_T1'
                                 */
  real_T DragForce_beta_w[2];          /* Mask Parameter: DragForce_beta_w
                                        * Referenced by:
                                        *   '<S44>/Cs'
                                        *   '<S44>/Cym'
                                        */
  real_T VehicleBody1DOFLongitudinal_g;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_g
                                 * Referenced by:
                                 *   '<S7>/g'
                                 *   '<S37>/Constant'
                                 */
  real_T VehicleBody1DOFLongitudinal_h;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_h
                                 * Referenced by:
                                 *   '<S7>/h'
                                 *   '<S18>/R_T3'
                                 *   '<S19>/R_T3'
                                 */
  real_T VehicleBody1DOFLongitudinal_m;
                                /* Mask Parameter: VehicleBody1DOFLongitudinal_m
                                 * Referenced by:
                                 *   '<S7>/1//m'
                                 *   '<S7>/m'
                                 *   '<S37>/Constant'
                                 *   '<S37>/Constant1'
                                 */
  real_T VehicleBody1DOFLongitudinal_x_o;
                              /* Mask Parameter: VehicleBody1DOFLongitudinal_x_o
                               * Referenced by: '<S4>/Integrator1'
                               */
  real_T VehicleBody1DOFLongitudinal_xdo;
                              /* Mask Parameter: VehicleBody1DOFLongitudinal_xdo
                               * Referenced by:
                               *   '<S7>/Integrator'
                               *   '<S15>/Integrator3'
                               */
  real_T Integrator_IC;                /* Expression: 0
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T FExtConstant_Value[3];        /* Expression: [0,0,0]
                                        * Referenced by: '<S3>/FExtConstant'
                                        */
  real_T MExtConstant_Value[3];        /* Expression: [0,0,0]
                                        * Referenced by: '<S3>/MExtConstant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S15>/Constant1'
                                        */
  real_T Constant11_Value;             /* Expression: 0
                                        * Referenced by: '<S15>/Constant11'
                                        */
  real_T Constant8_Value;              /* Expression: 0
                                        * Referenced by: '<S15>/Constant8'
                                        */
  real_T Constant4_Value[3];           /* Expression: ones(1,3)
                                        * Referenced by: '<S44>/Constant4'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S44>/Switch'
                                        */
  real_T Crm_tableData[2];             /* Expression: [0 0]
                                        * Referenced by: '<S44>/Crm'
                                        */
  real_T Crm_bp01Data[2];              /* Expression: [-1 1]
                                        * Referenced by: '<S44>/Crm'
                                        */
  real_T Constant10_Value;             /* Expression: 0
                                        * Referenced by: '<S16>/Constant10'
                                        */
  real_T Constant14_Value;             /* Expression: 0
                                        * Referenced by: '<S16>/Constant14'
                                        */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<S16>/Constant3'
                                        */
  real_T Constant4_Value_h;            /* Expression: 0
                                        * Referenced by: '<S16>/Constant4'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<S16>/Constant6'
                                        */
  real_T Constant8_Value_p;            /* Expression: 0
                                        * Referenced by: '<S16>/Constant8'
                                        */
  real_T Constant9_Value;              /* Expression: 0
                                        * Referenced by: '<S16>/Constant9'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S6>/Constant'
                                        */
  real_T Constant12_Value;             /* Expression: 0
                                        * Referenced by: '<S12>/Constant12'
                                        */
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<S12>/Constant2'
                                        */
  real_T Constant1_Value_l;            /* Expression: 0
                                        * Referenced by: '<S12>/Constant1'
                                        */
  real_T Constant_Value_m;             /* Expression: 0
                                        * Referenced by: '<S34>/Constant'
                                        */
  real_T Constant10_Value_o;           /* Expression: 0
                                        * Referenced by: '<S12>/Constant10'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_vehicleModel_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo *solverInfo;
  B_vehicleModel_T *blockIO;
  X_vehicleModel_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  ExtU_vehicleModel_T *inputs;
  ExtY_vehicleModel_T *outputs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[7];
  real_T odeF[3][7];
  ODE3_IntgData intgData;
  DW_vehicleModel_T *dwork;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_vehicleModel_T vehicleModel_P;

/* External data declarations for dependent source files */
extern const char *RT_MEMORY_ALLOCATION_ERROR;
extern P_vehicleModel_T vehicleModel_P;/* parameters */

/* Model entry point functions */
extern RT_MODEL_vehicleModel_T *vehicleModel(void);
extern void vehicleModel_initialize(RT_MODEL_vehicleModel_T *const
  vehicleModel_M);
extern void vehicleModel_step(RT_MODEL_vehicleModel_T *const vehicleModel_M);
extern void vehicleModel_terminate(RT_MODEL_vehicleModel_T * vehicleModel_M);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vehicleModel'
 * '<S1>'   : 'vehicleModel/Atomic Subsystem'
 * '<S2>'   : 'vehicleModel/Atomic Subsystem/Bicycle Kinematic Model'
 * '<S3>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal'
 * '<S4>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF'
 * '<S5>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation'
 * '<S6>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Inertial Axes'
 * '<S7>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Vehicle'
 * '<S8>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/WindDim'
 * '<S9>'   : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Body Frame'
 * '<S10>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Forces'
 * '<S11>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing'
 * '<S12>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Inertial Frame'
 * '<S13>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Moments'
 * '<S14>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power'
 * '<S15>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Body Frame/Longitudinal 1DOF'
 * '<S16>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Forces/Forces 1DOF'
 * '<S17>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF'
 * '<S18>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front'
 * '<S19>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear'
 * '<S20>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/Rotation Angles to Direction Cosine Matrix'
 * '<S21>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/transform to Inertial axes'
 * '<S22>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/transform to Inertial axes1'
 * '<S23>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/wxR'
 * '<S24>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S25>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/wxR/Subsystem'
 * '<S26>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Front/wxR/Subsystem1'
 * '<S27>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/Rotation Angles to Direction Cosine Matrix'
 * '<S28>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/transform to Inertial axes'
 * '<S29>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/transform to Inertial axes1'
 * '<S30>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/wxR'
 * '<S31>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S32>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/wxR/Subsystem'
 * '<S33>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Frame Routing/Longitudinal 1DOF/Hard Point Coordinate Transform Rear/wxR/Subsystem1'
 * '<S34>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Inertial Frame/Inertial Axes'
 * '<S35>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Inertial Frame/Inertial Axes/Rotation Angles to Direction Cosine Matrix'
 * '<S36>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Inertial Frame/Inertial Axes/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S37>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power/Longitudinal 1DOF'
 * '<S38>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power/Longitudinal 1DOF/Power Accounting Bus Creator'
 * '<S39>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power/Longitudinal 1DOF/Power Accounting Bus Creator/PwrNotTrnsfrd Input'
 * '<S40>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power/Longitudinal 1DOF/Power Accounting Bus Creator/PwrStored Input'
 * '<S41>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Bus Creation/Power/Longitudinal 1DOF/Power Accounting Bus Creator/PwrTrnsfrd Input'
 * '<S42>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Inertial Axes/Rotation Angles to Direction Cosine Matrix'
 * '<S43>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Inertial Axes/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S44>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/Vehicle/Drag Force'
 * '<S45>'  : 'vehicleModel/Atomic Subsystem/Vehicle Body 1DOF Longitudinal/Vehicle Body 1 DOF/WindDim/WindX'
 */
#endif                                 /* RTW_HEADER_vehicleModel_h_ */
