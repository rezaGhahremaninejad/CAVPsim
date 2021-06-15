/*
 * vehicleModel_types.h
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

#ifndef RTW_HEADER_vehicleModel_types_h_
#define RTW_HEADER_vehicleModel_types_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#ifndef struct_tag_seX5IO8aJqG3LSDcfXTWz2
#define struct_tag_seX5IO8aJqG3LSDcfXTWz2

struct tag_seX5IO8aJqG3LSDcfXTWz2
{
  uint32_T WheelBase;
  uint32_T VehicleSpeedRange;
  uint32_T MaxSteeringAngle;
  uint32_T VehicleInputs;
};

#endif                                 /*struct_tag_seX5IO8aJqG3LSDcfXTWz2*/

#ifndef typedef_seX5IO8aJqG3LSDcfXTWz2_vehicl_T
#define typedef_seX5IO8aJqG3LSDcfXTWz2_vehicl_T

typedef struct tag_seX5IO8aJqG3LSDcfXTWz2 seX5IO8aJqG3LSDcfXTWz2_vehicl_T;

#endif                               /*typedef_seX5IO8aJqG3LSDcfXTWz2_vehicl_T*/

#ifndef struct_tag_s8wGepGO9yku4SzkM4vzdIG
#define struct_tag_s8wGepGO9yku4SzkM4vzdIG

struct tag_s8wGepGO9yku4SzkM4vzdIG
{
  real_T WheelBase;
  real_T VehicleSpeedRange;
  real_T MaxSteeringAngle;
  real_T VehicleInputs;
};

#endif                                 /*struct_tag_s8wGepGO9yku4SzkM4vzdIG*/

#ifndef typedef_s8wGepGO9yku4SzkM4vzdIG_vehic_T
#define typedef_s8wGepGO9yku4SzkM4vzdIG_vehic_T

typedef struct tag_s8wGepGO9yku4SzkM4vzdIG s8wGepGO9yku4SzkM4vzdIG_vehic_T;

#endif                               /*typedef_s8wGepGO9yku4SzkM4vzdIG_vehic_T*/

#ifndef struct_tag_so4pBuofLXIy3ixtLNBoZq
#define struct_tag_so4pBuofLXIy3ixtLNBoZq

struct tag_so4pBuofLXIy3ixtLNBoZq
{
  char_T PartialMatching[6];
};

#endif                                 /*struct_tag_so4pBuofLXIy3ixtLNBoZq*/

#ifndef typedef_so4pBuofLXIy3ixtLNBoZq_vehicl_T
#define typedef_so4pBuofLXIy3ixtLNBoZq_vehicl_T

typedef struct tag_so4pBuofLXIy3ixtLNBoZq so4pBuofLXIy3ixtLNBoZq_vehicl_T;

#endif                               /*typedef_so4pBuofLXIy3ixtLNBoZq_vehicl_T*/

#ifndef typedef_c_cell_wrap_vehicleModel_T
#define typedef_c_cell_wrap_vehicleModel_T

typedef struct {
  uint32_T f1[8];
} c_cell_wrap_vehicleModel_T;

#endif                                 /*typedef_c_cell_wrap_vehicleModel_T*/

#ifndef typedef_d_cell_wrap_vehicleModel_T
#define typedef_d_cell_wrap_vehicleModel_T

typedef struct {
  char_T f1[25];
} d_cell_wrap_vehicleModel_T;

#endif                                 /*typedef_d_cell_wrap_vehicleModel_T*/

#ifndef typedef_bicycleKinematics_vehicleMode_T
#define typedef_bicycleKinematics_vehicleMode_T

typedef struct {
  real_T WheelBase;
  real_T VehicleSpeedRange[2];
  real_T MaxSteeringAngle;
  char_T VehicleInputsInternal[25];
} bicycleKinematics_vehicleMode_T;

#endif                               /*typedef_bicycleKinematics_vehicleMode_T*/

#ifndef typedef_robotics_slmobile_internal_bl_T
#define typedef_robotics_slmobile_internal_bl_T

typedef struct {
  boolean_T tunablePropertyChanged[3];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  c_cell_wrap_vehicleModel_T inputVarSize[3];
  bicycleKinematics_vehicleMode_T KinModel;
  real_T WheelBase;
  real_T VehicleSpeedRange[2];
  real_T MaxSteeringAngle;
} robotics_slmobile_internal_bl_T;

#endif                               /*typedef_robotics_slmobile_internal_bl_T*/

/* Parameters (default storage) */
typedef struct P_vehicleModel_T_ P_vehicleModel_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_vehicleModel_T RT_MODEL_vehicleModel_T;

#endif                                 /* RTW_HEADER_vehicleModel_types_h_ */
