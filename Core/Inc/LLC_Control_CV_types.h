/*
 * File: LLC_Control_CV_types.h
 *
 * Code generated for Simulink model 'LLC_Control_CV'.
 *
 * Model version                  : 1.4
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed Aug 13 14:42:44 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef LLC_Control_CV_types_h_
#define LLC_Control_CV_types_h_
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_ThresholdsBus_
#define DEFINED_TYPEDEF_FOR_ThresholdsBus_

typedef struct {
  float MaxCurrentLimit_A;
  float OTWarningLimit_C;
  float OTErrorLimit_C;
  float UVWarningLimit_LLC_Out_V;
  float UVErrorLimit_LLC_Out_V;
  float OVWarningLimit_LLC_Out_V;
  float OVErrorLimit_LLC_Out_V;
  float UVWarningLimit_LLC_In_V;
  float UVErrorLimit_LLC_In_V;
  float OVWarningLimit_LLC_In_V;
  float OVErrorLimit_LLC_In_V;
  float OCWarningLimit_A;
  float OCErrorLimit_A;
  float VoltageProtectionTimeout_msec;
  float TempProtectionTimeout_msec;
  float CurrentProtectionTimeout_msec;
} ThresholdsBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CurrentState_
#define DEFINED_TYPEDEF_FOR_CurrentState_

typedef enum {
  SafeCurrent = 0,                     /* Default value */
  OC_Warning,
  OC_Error
} CurrentState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TempState_
#define DEFINED_TYPEDEF_FOR_TempState_

typedef enum {
  SafeTemperature = 0,                 /* Default value */
  OT_Warning,
  OT_Error
} TempState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VoltageState_
#define DEFINED_TYPEDEF_FOR_VoltageState_

typedef enum {
  SafeVoltage = 0,                     /* Default value */
  OV_Warning,
  OV_Error,
  UV_Warning,
  UV_Error
} VoltageState;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_LLC_Control_CV_T RT_MODEL_LLC_Control_CV_T;

#endif                                 /* LLC_Control_CV_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
