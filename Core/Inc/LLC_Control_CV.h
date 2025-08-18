/*
 * File: LLC_Control_CV.h
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

#ifndef LLC_Control_CV_h_
#define LLC_Control_CV_h_
#ifndef LLC_Control_CV_COMMON_INCLUDES_
#define LLC_Control_CV_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "math.h"
#endif                                 /* LLC_Control_CV_COMMON_INCLUDES_ */

#include "LLC_Control_CV_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  double Integrator_DSTATE;            /* '<S40>/Integrator' */
  uint32_t durationCounter_1;          /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_i;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_h;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2;          /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_g;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_o;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_p;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_a;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_f;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_b;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_k;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_g;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_p4;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_p;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_i;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_e;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_e;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_ej;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_a;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_h;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_gz;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_pd;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_h1;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_ia;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_ey;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_d;        /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_eh;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_1_pj;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_i5;       /* '<S1>/Protection_States' */
  uint32_t durationCounter_2_k;        /* '<S1>/Protection_States' */
  uint8_t Integrator_IC_LOADING;       /* '<S40>/Integrator' */
  uint8_t is_active_c7_LLC_Control_CV; /* '<S1>/Protection_States' */
  uint8_t is_VoltageProtection_LLC_Out;/* '<S1>/Protection_States' */
  uint8_t is_CurrentProtection;        /* '<S1>/Protection_States' */
  uint8_t is_TemperatureProtection;    /* '<S1>/Protection_States' */
  uint8_t is_VoltageProtection_LLC_In; /* '<S1>/Protection_States' */
} DW_LLC_Control_CV_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  double V_out_LLC;                    /* '<Root>/V_out_LLC' */
  double V_ref;                        /* '<Root>/V_ref' */
  double Kp;                           /* '<Root>/Kp' */
  double Ki;                           /* '<Root>/Ki' */
  double Freq_up_limit;                /* '<Root>/Freq_up_limit' */
  double Freq_low_limit;               /* '<Root>/Freq_low_limit' */
  double V_ref_up_limit;               /* '<Root>/V_ref_up_limit' */
  double V_ref_low_limit;              /* '<Root>/V_ref_low_limit' */
  double I_out_LLC;                    /* '<Root>/I_out_LLC' */
  double V_in_LLC;                     /* '<Root>/V_in_LLC' */
  double OBC_Temp_C;                   /* '<Root>/OBC_Temp_C' */
  ThresholdsBus Thresholds;            /* '<Root>/Thresholds' */
} ExtU_LLC_Control_CV_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  double fs;                           /* '<Root>/fs' */
  CurrentState CurrentFlag;            /* '<Root>/CurrentFlag' */
  TempState TempFlag;                  /* '<Root>/TempFlag' */
  VoltageState VoltageFlag_LLC_Out;    /* '<Root>/VoltageFlag_LLC_Out' */
  VoltageState VoltageFlag_LLC_In;     /* '<Root>/VoltageFlag_LLC_In' */
} ExtY_LLC_Control_CV_T;

/* Real-time Model Data Structure */
struct tag_RTM_LLC_Control_CV_T {
  const char * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_LLC_Control_CV_T LLC_Control_CV_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_LLC_Control_CV_T LLC_Control_CV_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LLC_Control_CV_T LLC_Control_CV_Y;

/* Model entry point functions */
extern void LLC_Control_CV_initialize(void);
extern void LLC_Control_CV_step(void);
extern void LLC_Control_CV_terminate(void);

/* Real-time Model object */
extern RT_MODEL_LLC_Control_CV_T *const LLC_Control_CV_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Propagation' : Unused code path elimination
 * Block '<S4>/Display1' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('LLC_logic/LLC_Control_CV')    - opens subsystem LLC_logic/LLC_Control_CV
 * hilite_system('LLC_logic/LLC_Control_CV/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LLC_logic'
 * '<S1>'   : 'LLC_logic/LLC_Control_CV'
 * '<S2>'   : 'LLC_logic/LLC_Control_CV/Protection_States'
 * '<S3>'   : 'LLC_logic/LLC_Control_CV/Saturation Dynamic'
 * '<S4>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator'
 * '<S5>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller'
 * '<S6>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Anti-windup'
 * '<S7>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/D Gain'
 * '<S8>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/External Derivative'
 * '<S9>'   : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Filter'
 * '<S10>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Filter ICs'
 * '<S11>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/I Gain'
 * '<S12>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Ideal P Gain'
 * '<S13>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Ideal P Gain Fdbk'
 * '<S14>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Integrator'
 * '<S15>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Integrator ICs'
 * '<S16>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/N Copy'
 * '<S17>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/N Gain'
 * '<S18>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/P Copy'
 * '<S19>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Parallel P Gain'
 * '<S20>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Reset Signal'
 * '<S21>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Saturation'
 * '<S22>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Saturation Fdbk'
 * '<S23>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Sum'
 * '<S24>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Sum Fdbk'
 * '<S25>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tracking Mode'
 * '<S26>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tracking Mode Sum'
 * '<S27>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tsamp - Integral'
 * '<S28>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tsamp - Ngain'
 * '<S29>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/postSat Signal'
 * '<S30>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/preInt Signal'
 * '<S31>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/preSat Signal'
 * '<S32>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Anti-windup/Passthrough'
 * '<S33>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/D Gain/Disabled'
 * '<S34>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/External Derivative/Disabled'
 * '<S35>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Filter/Disabled'
 * '<S36>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Filter ICs/Disabled'
 * '<S37>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/I Gain/External Parameters'
 * '<S38>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Ideal P Gain/Passthrough'
 * '<S39>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S40>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Integrator/Discrete'
 * '<S41>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Integrator ICs/External IC'
 * '<S42>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S43>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/N Gain/Disabled'
 * '<S44>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/P Copy/Disabled'
 * '<S45>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Parallel P Gain/External Parameters'
 * '<S46>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Reset Signal/Disabled'
 * '<S47>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Saturation/External'
 * '<S48>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S49>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Saturation Fdbk/Disabled'
 * '<S50>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Sum/Sum_PI'
 * '<S51>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Sum Fdbk/Disabled'
 * '<S52>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tracking Mode/Disabled'
 * '<S53>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S54>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S55>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S56>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/postSat Signal/Forward_Path'
 * '<S57>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/preInt Signal/Internal PreInt'
 * '<S58>'  : 'LLC_logic/LLC_Control_CV/Voltage Regulator/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* LLC_Control_CV_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
