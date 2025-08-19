/*
 * File: LLC_Control_CV.c
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

#include "LLC_Control_CV.h"
#include <stdint.h>
#include "LLC_Control_CV_types.h"
#include "main.h"

/* Named constants for Chart: '<S1>/Protection_States' */
#define LLC_Control_CV_IN_CurrentSafe  ((uint8_t)1U)
#define LLC_Control_CV_IN_HighTempError ((uint8_t)1U)
#define LLC_Control_CV_IN_OC_Error     ((uint8_t)2U)
#define LLC_Control_CV_IN_OC_Warning   ((uint8_t)3U)
#define LLC_Control_CV_IN_OV_Error     ((uint8_t)1U)
#define LLC_Control_CV_IN_OV_Warning   ((uint8_t)2U)
#define LLC_Control_CV_IN_TempSafe     ((uint8_t)3U)
#define LLC_Control_CV_IN_UV_Error     ((uint8_t)3U)
#define LLC_Control_CV_IN_UV_Warning   ((uint8_t)4U)
#define LLC_Control_CV_IN_VoltageSafe  ((uint8_t)5U)
#define LLC_Control__IN_HighTempWarning ((uint8_t)2U)

/* Block states (default storage) */
DW_LLC_Control_CV_T LLC_Control_CV_DW;

/* External inputs (root inport signals with default storage) */
ExtU_LLC_Control_CV_T LLC_Control_CV_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LLC_Control_CV_T LLC_Control_CV_Y;

/* Real-time model */
static RT_MODEL_LLC_Control_CV_T LLC_Control_CV_M_;
RT_MODEL_LLC_Control_CV_T *const LLC_Control_CV_M = &LLC_Control_CV_M_;

/* Forward declaration for local functions */
static void LLC_C_VoltageProtection_LLC_Out(void);
static void LLC_Contr_TemperatureProtection(void);
static void LLC_Co_VoltageProtection_LLC_In(void);

/* Function for Chart: '<S1>/Protection_States' */
//__attribute__((section(".ccmram")))
static void LLC_C_VoltageProtection_LLC_Out(void)
{
  double tmp;
  switch (LLC_Control_CV_DW.is_VoltageProtection_LLC_Out) {
   case LLC_Control_CV_IN_OV_Error:
    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = OV_Error;

    /* Inport: '<Root>/V_out_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_out_LLC <
          LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V)) {
      LLC_Control_CV_DW.durationCounter_1_e = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_out_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_e > tmp) {
      LLC_Control_CV_DW.durationCounter_2_e = 0U;
      LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
        LLC_Control_CV_IN_VoltageSafe;

      /* Outport: '<Root>/VoltageFlag_LLC_Out' */
      LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;
    } else {
      if (!(LLC_Control_CV_U.V_out_LLC <
            LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_Out_V)) {
        LLC_Control_CV_DW.durationCounter_2_a = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_a > tmp) {
        LLC_Control_CV_DW.durationCounter_2_h = 0U;
        LLC_Control_CV_DW.durationCounter_1_ej = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
          LLC_Control_CV_IN_OV_Warning;

        /* Outport: '<Root>/VoltageFlag_LLC_Out' */
        LLC_Control_CV_Y.VoltageFlag_LLC_Out = OV_Warning;
      }
    }
    break;

   case LLC_Control_CV_IN_OV_Warning:
    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = OV_Warning;

    /* Inport: '<Root>/V_out_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_out_LLC >
          LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_Out_V)) {
      LLC_Control_CV_DW.durationCounter_2_h = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_out_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_2_h > tmp) {
      LLC_Control_CV_DW.durationCounter_2_a = 0U;
      LLC_Control_CV_DW.durationCounter_1_e = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
        LLC_Control_CV_IN_OV_Error;

      /* Outport: '<Root>/VoltageFlag_LLC_Out' */
      LLC_Control_CV_Y.VoltageFlag_LLC_Out = OV_Error;
    } else {
      if (!(LLC_Control_CV_U.V_out_LLC <
            LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V)) {
        LLC_Control_CV_DW.durationCounter_1_ej = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_1_ej > tmp) {
        LLC_Control_CV_DW.durationCounter_2_e = 0U;
        LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_Out' */
        LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;
      }
    }
    break;

   case LLC_Control_CV_IN_UV_Error:
    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = UV_Error;

    /* Inport: '<Root>/V_out_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_out_LLC >
          LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_Out_V)) {
      LLC_Control_CV_DW.durationCounter_1_k = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_out_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_k > tmp) {
      LLC_Control_CV_DW.durationCounter_2_p = 0U;
      LLC_Control_CV_DW.durationCounter_1_g = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
        LLC_Control_CV_IN_UV_Warning;

      /* Outport: '<Root>/VoltageFlag_LLC_Out' */
      LLC_Control_CV_Y.VoltageFlag_LLC_Out = UV_Warning;
    } else {
      if (!(LLC_Control_CV_U.V_out_LLC >
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V)) {
        LLC_Control_CV_DW.durationCounter_2_i = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_i > tmp) {
        LLC_Control_CV_DW.durationCounter_2_e = 0U;
        LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_Out' */
        LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;
      }
    }
    break;

   case LLC_Control_CV_IN_UV_Warning:
    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = UV_Warning;

    /* Inport: '<Root>/V_out_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_out_LLC <
          LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_Out_V)) {
      LLC_Control_CV_DW.durationCounter_1_g = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_out_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_g > tmp) {
      LLC_Control_CV_DW.durationCounter_2_i = 0U;
      LLC_Control_CV_DW.durationCounter_1_k = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
        LLC_Control_CV_IN_UV_Error;

      /* Outport: '<Root>/VoltageFlag_LLC_Out' */
      LLC_Control_CV_Y.VoltageFlag_LLC_Out = UV_Error;
    } else {
      if (!(LLC_Control_CV_U.V_out_LLC >
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V)) {
        LLC_Control_CV_DW.durationCounter_2_p = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_p > tmp) {
        LLC_Control_CV_DW.durationCounter_2_e = 0U;
        LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_Out' */
        LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;
      }
    }
    break;

   default:
    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    /* case IN_VoltageSafe: */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;

    /* Inport: '<Root>/V_out_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_out_LLC >
          LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V)) {
      LLC_Control_CV_DW.durationCounter_2_e = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_out_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_2_e > tmp) {
      LLC_Control_CV_DW.durationCounter_2_h = 0U;
      LLC_Control_CV_DW.durationCounter_1_ej = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
        LLC_Control_CV_IN_OV_Warning;

      /* Outport: '<Root>/VoltageFlag_LLC_Out' */
      LLC_Control_CV_Y.VoltageFlag_LLC_Out = OV_Warning;
    } else {
      if (!(LLC_Control_CV_U.V_out_LLC <
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V)) {
        LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_1_p4 > tmp) {
        LLC_Control_CV_DW.durationCounter_2_p = 0U;
        LLC_Control_CV_DW.durationCounter_1_g = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
          LLC_Control_CV_IN_UV_Warning;

        /* Outport: '<Root>/VoltageFlag_LLC_Out' */
        LLC_Control_CV_Y.VoltageFlag_LLC_Out = UV_Warning;
      }
    }
    break;
  }
}

/* Function for Chart: '<S1>/Protection_States' */
//__attribute__((section(".ccmram")))
static void LLC_Contr_TemperatureProtection(void)
{
  double tmp;
  switch (LLC_Control_CV_DW.is_TemperatureProtection) {
   case LLC_Control_CV_IN_HighTempError:
    /* Outport: '<Root>/TempFlag' */
    LLC_Control_CV_Y.TempFlag = OT_Error;

    /* Inport: '<Root>/OBC_Temp_C' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.OBC_Temp_C <
          LLC_Control_CV_U.Thresholds.OTWarningLimit_C)) {
      LLC_Control_CV_DW.durationCounter_1_i = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/OBC_Temp_C' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    tmp = LLC_Control_CV_U.Thresholds.TempProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_i > tmp) {
      LLC_Control_CV_DW.durationCounter_1 = 0U;
      LLC_Control_CV_DW.is_TemperatureProtection = LLC_Control_CV_IN_TempSafe;

      /* Outport: '<Root>/TempFlag' */
      LLC_Control_CV_Y.TempFlag = SafeTemperature;
    } else {
      if (!(LLC_Control_CV_U.OBC_Temp_C <
            LLC_Control_CV_U.Thresholds.OTErrorLimit_C)) {
        LLC_Control_CV_DW.durationCounter_2 = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2 > tmp) {
        LLC_Control_CV_DW.durationCounter_2_g = 0U;
        LLC_Control_CV_DW.durationCounter_1_h = 0U;
        LLC_Control_CV_DW.is_TemperatureProtection =
          LLC_Control__IN_HighTempWarning;

        /* Outport: '<Root>/TempFlag' */
        LLC_Control_CV_Y.TempFlag = OT_Warning;
      }
    }
    break;

   case LLC_Control__IN_HighTempWarning:
    /* Outport: '<Root>/TempFlag' */
    LLC_Control_CV_Y.TempFlag = OT_Warning;

    /* Inport: '<Root>/OBC_Temp_C' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.OBC_Temp_C >
          LLC_Control_CV_U.Thresholds.OTErrorLimit_C)) {
      LLC_Control_CV_DW.durationCounter_2_g = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/OBC_Temp_C' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    tmp = LLC_Control_CV_U.Thresholds.TempProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_2_g > tmp) {
      LLC_Control_CV_DW.durationCounter_2 = 0U;
      LLC_Control_CV_DW.durationCounter_1_i = 0U;
      LLC_Control_CV_DW.is_TemperatureProtection =
        LLC_Control_CV_IN_HighTempError;

      /* Outport: '<Root>/TempFlag' */
      LLC_Control_CV_Y.TempFlag = OT_Error;
    } else {
      if (!(LLC_Control_CV_U.OBC_Temp_C <
            LLC_Control_CV_U.Thresholds.OTWarningLimit_C)) {
        LLC_Control_CV_DW.durationCounter_1_h = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_1_h > tmp) {
        LLC_Control_CV_DW.durationCounter_1 = 0U;
        LLC_Control_CV_DW.is_TemperatureProtection = LLC_Control_CV_IN_TempSafe;

        /* Outport: '<Root>/TempFlag' */
        LLC_Control_CV_Y.TempFlag = SafeTemperature;
      }
    }
    break;

   default:
    /* Outport: '<Root>/TempFlag' */
    /* case IN_TempSafe: */
    LLC_Control_CV_Y.TempFlag = SafeTemperature;

    /* Inport: '<Root>/OBC_Temp_C' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.OBC_Temp_C >
          LLC_Control_CV_U.Thresholds.OTWarningLimit_C)) {
      LLC_Control_CV_DW.durationCounter_1 = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    if ((double)LLC_Control_CV_DW.durationCounter_1 >
        LLC_Control_CV_U.Thresholds.TempProtectionTimeout_msec * 50.0F) {
      LLC_Control_CV_DW.durationCounter_2_g = 0U;
      LLC_Control_CV_DW.durationCounter_1_h = 0U;
      LLC_Control_CV_DW.is_TemperatureProtection =
        LLC_Control__IN_HighTempWarning;

      /* Outport: '<Root>/TempFlag' */
      LLC_Control_CV_Y.TempFlag = OT_Warning;
    }
    break;
  }
}

/* Function for Chart: '<S1>/Protection_States' */
//__attribute__((section(".ccmram")))
static void LLC_Co_VoltageProtection_LLC_In(void)
{
  double tmp;
  switch (LLC_Control_CV_DW.is_VoltageProtection_LLC_In) {
   case LLC_Control_CV_IN_OV_Error:
    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = OV_Error;

    /* Inport: '<Root>/V_in_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_in_LLC <
          LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V)) {
      LLC_Control_CV_DW.durationCounter_1_eh = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_in_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_eh > tmp) {
      LLC_Control_CV_DW.durationCounter_2_d = 0U;
      LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
        LLC_Control_CV_IN_VoltageSafe;

      /* Outport: '<Root>/VoltageFlag_LLC_In' */
      LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;
    } else {
      if (!(LLC_Control_CV_U.V_in_LLC <
            LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_In_V)) {
        LLC_Control_CV_DW.durationCounter_2_i5 = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_i5 > tmp) {
        LLC_Control_CV_DW.durationCounter_2_k = 0U;
        LLC_Control_CV_DW.durationCounter_1_pj = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
          LLC_Control_CV_IN_OV_Warning;

        /* Outport: '<Root>/VoltageFlag_LLC_In' */
        LLC_Control_CV_Y.VoltageFlag_LLC_In = OV_Warning;
      }
    }
    break;

   case LLC_Control_CV_IN_OV_Warning:
    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = OV_Warning;

    /* Inport: '<Root>/V_in_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_in_LLC >
          LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_In_V)) {
      LLC_Control_CV_DW.durationCounter_2_k = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_in_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_2_k > tmp) {
      LLC_Control_CV_DW.durationCounter_2_i5 = 0U;
      LLC_Control_CV_DW.durationCounter_1_eh = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_In = LLC_Control_CV_IN_OV_Error;

      /* Outport: '<Root>/VoltageFlag_LLC_In' */
      LLC_Control_CV_Y.VoltageFlag_LLC_In = OV_Error;
    } else {
      if (!(LLC_Control_CV_U.V_in_LLC <
            LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V)) {
        LLC_Control_CV_DW.durationCounter_1_pj = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_1_pj > tmp) {
        LLC_Control_CV_DW.durationCounter_2_d = 0U;
        LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_In' */
        LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;
      }
    }
    break;

   case LLC_Control_CV_IN_UV_Error:
    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = UV_Error;

    /* Inport: '<Root>/V_in_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_in_LLC >
          LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_In_V)) {
      LLC_Control_CV_DW.durationCounter_1_gz = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_in_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_gz > tmp) {
      LLC_Control_CV_DW.durationCounter_2_ia = 0U;
      LLC_Control_CV_DW.durationCounter_1_pd = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
        LLC_Control_CV_IN_UV_Warning;

      /* Outport: '<Root>/VoltageFlag_LLC_In' */
      LLC_Control_CV_Y.VoltageFlag_LLC_In = UV_Warning;
    } else {
      if (!(LLC_Control_CV_U.V_in_LLC >
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V)) {
        LLC_Control_CV_DW.durationCounter_2_ey = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_ey > tmp) {
        LLC_Control_CV_DW.durationCounter_2_d = 0U;
        LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_In' */
        LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;
      }
    }
    break;

   case LLC_Control_CV_IN_UV_Warning:
    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = UV_Warning;

    /* Inport: '<Root>/V_in_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_in_LLC <
          LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_In_V)) {
      LLC_Control_CV_DW.durationCounter_1_pd = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_in_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_1_pd > tmp) {
      LLC_Control_CV_DW.durationCounter_2_ey = 0U;
      LLC_Control_CV_DW.durationCounter_1_gz = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_In = LLC_Control_CV_IN_UV_Error;

      /* Outport: '<Root>/VoltageFlag_LLC_In' */
      LLC_Control_CV_Y.VoltageFlag_LLC_In = UV_Error;
    } else {
      if (!(LLC_Control_CV_U.V_in_LLC >
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V)) {
        LLC_Control_CV_DW.durationCounter_2_ia = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_2_ia > tmp) {
        LLC_Control_CV_DW.durationCounter_2_d = 0U;
        LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
          LLC_Control_CV_IN_VoltageSafe;

        /* Outport: '<Root>/VoltageFlag_LLC_In' */
        LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;
      }
    }
    break;

   default:
    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    /* case IN_VoltageSafe: */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;

    /* Inport: '<Root>/V_in_LLC' incorporates:
     *  Inport: '<Root>/Thresholds'
     */
    if (!(LLC_Control_CV_U.V_in_LLC >
          LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V)) {
      LLC_Control_CV_DW.durationCounter_2_d = 0U;
    }

    /* Inport: '<Root>/Thresholds' */
    /* Inport: '<Root>/Thresholds' incorporates:
     *  Inport: '<Root>/V_in_LLC'
     */
    tmp = LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec * 50.0F;
    if (LLC_Control_CV_DW.durationCounter_2_d > tmp) {
      LLC_Control_CV_DW.durationCounter_2_k = 0U;
      LLC_Control_CV_DW.durationCounter_1_pj = 0U;
      LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
        LLC_Control_CV_IN_OV_Warning;

      /* Outport: '<Root>/VoltageFlag_LLC_In' */
      LLC_Control_CV_Y.VoltageFlag_LLC_In = OV_Warning;
    } else {
      if (!(LLC_Control_CV_U.V_in_LLC <
            LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V)) {
        LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
      }

      if (LLC_Control_CV_DW.durationCounter_1_h1 > tmp) {
        LLC_Control_CV_DW.durationCounter_2_ia = 0U;
        LLC_Control_CV_DW.durationCounter_1_pd = 0U;
        LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
          LLC_Control_CV_IN_UV_Warning;

        /* Outport: '<Root>/VoltageFlag_LLC_In' */
        LLC_Control_CV_Y.VoltageFlag_LLC_In = UV_Warning;
      }
    }
    break;
  }
}

/* Model step function */
__attribute__((section(".ccmram"))) 
void LLC_Control_CV_step(void)
{
  double rtb_Integrator;

  /* Outputs for Atomic SubSystem: '<Root>/LLC_Control_CV' */
  /* Chart: '<S1>/Protection_States' incorporates:
   *  Inport: '<Root>/I_out_LLC'
   *  Inport: '<Root>/OBC_Temp_C'
   *  Inport: '<Root>/Thresholds'
   *  Inport: '<Root>/V_in_LLC'
   *  Inport: '<Root>/V_out_LLC'
   */
  if (LLC_Control_CV_DW.is_active_c7_LLC_Control_CV == 0) {
    LLC_Control_CV_DW.is_active_c7_LLC_Control_CV = 1U;
    LLC_Control_CV_DW.durationCounter_2_e = 0U;
    LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
    LLC_Control_CV_DW.is_VoltageProtection_LLC_Out =
      LLC_Control_CV_IN_VoltageSafe;

    /* Outport: '<Root>/VoltageFlag_LLC_Out' */
    LLC_Control_CV_Y.VoltageFlag_LLC_Out = SafeVoltage;
    LLC_Control_CV_DW.durationCounter_1_o = 0U;
    LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_CurrentSafe;

    /* Outport: '<Root>/CurrentFlag' */
    LLC_Control_CV_Y.CurrentFlag = SafeCurrent;
    LLC_Control_CV_DW.durationCounter_1 = 0U;
    LLC_Control_CV_DW.is_TemperatureProtection = LLC_Control_CV_IN_TempSafe;

    /* Outport: '<Root>/TempFlag' */
    LLC_Control_CV_Y.TempFlag = SafeTemperature;
    LLC_Control_CV_DW.durationCounter_2_d = 0U;
    LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
    LLC_Control_CV_DW.is_VoltageProtection_LLC_In =
      LLC_Control_CV_IN_VoltageSafe;

    /* Outport: '<Root>/VoltageFlag_LLC_In' */
    LLC_Control_CV_Y.VoltageFlag_LLC_In = SafeVoltage;
  } else {
   LLC_C_VoltageProtection_LLC_Out();
    switch (LLC_Control_CV_DW.is_CurrentProtection) {
     case LLC_Control_CV_IN_CurrentSafe:
      /* Outport: '<Root>/CurrentFlag' */
      LLC_Control_CV_Y.CurrentFlag = SafeCurrent;
      if (!(LLC_Control_CV_U.I_out_LLC >
            LLC_Control_CV_U.Thresholds.OCWarningLimit_A)) {
        LLC_Control_CV_DW.durationCounter_1_o = 0U;
      }

      if ((double)LLC_Control_CV_DW.durationCounter_1_o >
          LLC_Control_CV_U.Thresholds.CurrentProtectionTimeout_msec * 50.0F) {
        LLC_Control_CV_DW.durationCounter_2_f = 0U;
        LLC_Control_CV_DW.durationCounter_1_a = 0U;
        LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_OC_Warning;

        /* Outport: '<Root>/CurrentFlag' */
        LLC_Control_CV_Y.CurrentFlag = OC_Warning;
      }
      break;

     case LLC_Control_CV_IN_OC_Error:
      /* Outport: '<Root>/CurrentFlag' */
      LLC_Control_CV_Y.CurrentFlag = OC_Error;
      if (!(LLC_Control_CV_U.I_out_LLC <
            LLC_Control_CV_U.Thresholds.OCWarningLimit_A)) {
        LLC_Control_CV_DW.durationCounter_1_p = 0U;
      }

      rtb_Integrator = LLC_Control_CV_U.Thresholds.CurrentProtectionTimeout_msec
        * 50.0F;
      if (LLC_Control_CV_DW.durationCounter_1_p > rtb_Integrator) {
        LLC_Control_CV_DW.durationCounter_1_o = 0U;
        LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_CurrentSafe;

        /* Outport: '<Root>/CurrentFlag' */
        LLC_Control_CV_Y.CurrentFlag = SafeCurrent;
      } else {
        if (!(LLC_Control_CV_U.I_out_LLC <
              LLC_Control_CV_U.Thresholds.OCErrorLimit_A)) {
          LLC_Control_CV_DW.durationCounter_2_b = 0U;
        }

        if (LLC_Control_CV_DW.durationCounter_2_b > rtb_Integrator) {
          LLC_Control_CV_DW.durationCounter_2_f = 0U;
          LLC_Control_CV_DW.durationCounter_1_a = 0U;
          LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_OC_Warning;

          /* Outport: '<Root>/CurrentFlag' */
          LLC_Control_CV_Y.CurrentFlag = OC_Warning;
        }
      }
      break;

     default:
      /* Outport: '<Root>/CurrentFlag' */
      /* case IN_OC_Warning: */
      LLC_Control_CV_Y.CurrentFlag = OC_Warning;
      if (!(LLC_Control_CV_U.I_out_LLC >
            LLC_Control_CV_U.Thresholds.OCErrorLimit_A)) {
        LLC_Control_CV_DW.durationCounter_2_f = 0U;
      }

      rtb_Integrator = LLC_Control_CV_U.Thresholds.CurrentProtectionTimeout_msec
        * 50.0F;
      if (LLC_Control_CV_DW.durationCounter_2_f > rtb_Integrator) {
        LLC_Control_CV_DW.durationCounter_2_b = 0U;
        LLC_Control_CV_DW.durationCounter_1_p = 0U;
        LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_OC_Error;

        /* Outport: '<Root>/CurrentFlag' */
        LLC_Control_CV_Y.CurrentFlag = OC_Error;
      } else {
        if (!(LLC_Control_CV_U.I_out_LLC <
              LLC_Control_CV_U.Thresholds.OCWarningLimit_A)) {
          LLC_Control_CV_DW.durationCounter_1_a = 0U;
        }

        if (LLC_Control_CV_DW.durationCounter_1_a > rtb_Integrator) {
          LLC_Control_CV_DW.durationCounter_1_o = 0U;
          LLC_Control_CV_DW.is_CurrentProtection = LLC_Control_CV_IN_CurrentSafe;

          /* Outport: '<Root>/CurrentFlag' */
          LLC_Control_CV_Y.CurrentFlag = SafeCurrent;
        }
      }
      break;
    }

   LLC_Contr_TemperatureProtection();
   LLC_Co_VoltageProtection_LLC_In();
  }

  if (LLC_Control_CV_U.OBC_Temp_C > LLC_Control_CV_U.Thresholds.OTWarningLimit_C)
  {
    LLC_Control_CV_DW.durationCounter_1++;
  } else {
    LLC_Control_CV_DW.durationCounter_1 = 0U;
  }

  if (LLC_Control_CV_U.OBC_Temp_C < LLC_Control_CV_U.Thresholds.OTWarningLimit_C)
  {
    LLC_Control_CV_DW.durationCounter_1_i++;
    LLC_Control_CV_DW.durationCounter_1_h++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_i = 0U;
    LLC_Control_CV_DW.durationCounter_1_h = 0U;
  }

  if (LLC_Control_CV_U.OBC_Temp_C < LLC_Control_CV_U.Thresholds.OTErrorLimit_C)
  {
    LLC_Control_CV_DW.durationCounter_2++;
  } else {
    LLC_Control_CV_DW.durationCounter_2 = 0U;
  }

  if (LLC_Control_CV_U.OBC_Temp_C > LLC_Control_CV_U.Thresholds.OTErrorLimit_C)
  {
    LLC_Control_CV_DW.durationCounter_2_g++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_g = 0U;
  }

  if (LLC_Control_CV_U.I_out_LLC > LLC_Control_CV_U.Thresholds.OCWarningLimit_A)
  {
    LLC_Control_CV_DW.durationCounter_1_o++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_o = 0U;
  }

  if (LLC_Control_CV_U.I_out_LLC < LLC_Control_CV_U.Thresholds.OCWarningLimit_A)
  {
    LLC_Control_CV_DW.durationCounter_1_p++;
    LLC_Control_CV_DW.durationCounter_1_a++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_p = 0U;
    LLC_Control_CV_DW.durationCounter_1_a = 0U;
  }

  if (LLC_Control_CV_U.I_out_LLC > LLC_Control_CV_U.Thresholds.OCErrorLimit_A) {
    LLC_Control_CV_DW.durationCounter_2_f++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_f = 0U;
  }

  if (LLC_Control_CV_U.I_out_LLC < LLC_Control_CV_U.Thresholds.OCErrorLimit_A) {
    LLC_Control_CV_DW.durationCounter_2_b++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_b = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC >
      LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_1_k++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_k = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC <
      LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_1_g++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_g = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC <
      LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_1_p4++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_p4 = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC >
      LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_2_p++;
    LLC_Control_CV_DW.durationCounter_2_i++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_p = 0U;
    LLC_Control_CV_DW.durationCounter_2_i = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC >
      LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_2_e++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_e = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC <
      LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_1_e++;
    LLC_Control_CV_DW.durationCounter_1_ej++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_e = 0U;
    LLC_Control_CV_DW.durationCounter_1_ej = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC <
      LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_2_a++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_a = 0U;
  }

  if (LLC_Control_CV_U.V_out_LLC >
      LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_Out_V) {
    LLC_Control_CV_DW.durationCounter_2_h++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_h = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC >
      LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_1_gz++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_gz = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC <
      LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_1_pd++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_pd = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC <
      LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_1_h1++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_h1 = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC >
      LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_2_ia++;
    LLC_Control_CV_DW.durationCounter_2_ey++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_ia = 0U;
    LLC_Control_CV_DW.durationCounter_2_ey = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC >
      LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_2_d++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_d = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC <
      LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_1_eh++;
    LLC_Control_CV_DW.durationCounter_1_pj++;
  } else {
    LLC_Control_CV_DW.durationCounter_1_eh = 0U;
    LLC_Control_CV_DW.durationCounter_1_pj = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC <
      LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_2_i5++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_i5 = 0U;
  }

  if (LLC_Control_CV_U.V_in_LLC >
      LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_In_V) {
    LLC_Control_CV_DW.durationCounter_2_k++;
  } else {
    LLC_Control_CV_DW.durationCounter_2_k = 0U;
  }

  /* End of Chart: '<S1>/Protection_States' */

  /* Switch: '<S3>/Switch2' incorporates:
   *  Inport: '<Root>/V_ref'
   *  Inport: '<Root>/V_ref_low_limit'
   *  Inport: '<Root>/V_ref_up_limit'
   *  RelationalOperator: '<S3>/LowerRelop1'
   *  RelationalOperator: '<S3>/UpperRelop'
   *  Switch: '<S3>/Switch'
   */
  if (LLC_Control_CV_U.V_ref > LLC_Control_CV_U.V_ref_up_limit) {
    rtb_Integrator = LLC_Control_CV_U.V_ref_up_limit;
  } else if (LLC_Control_CV_U.V_ref < LLC_Control_CV_U.V_ref_low_limit) {
    /* Switch: '<S3>/Switch' incorporates:
     *  Inport: '<Root>/V_ref_low_limit'
     */
    rtb_Integrator = LLC_Control_CV_U.V_ref_low_limit;
  } else {
    rtb_Integrator = LLC_Control_CV_U.V_ref;
  }

  /* Sum: '<S4>/Sum1' incorporates:
   *  Inport: '<Root>/V_out_LLC'
   *  Switch: '<S3>/Switch2'
   */
  rtb_Integrator = LLC_Control_CV_U.V_out_LLC - rtb_Integrator;

  /* DiscreteIntegrator: '<S40>/Integrator' incorporates:
   *  Gain: '<S4>/Gain'
   *  Inport: '<Root>/Freq_low_limit'
   *  Inport: '<Root>/Freq_up_limit'
   *  Sum: '<S4>/Add'
   */
  if (LLC_Control_CV_DW.Integrator_IC_LOADING != 0) {
    LLC_Control_CV_DW.Integrator_DSTATE = (LLC_Control_CV_U.Freq_up_limit +
      LLC_Control_CV_U.Freq_low_limit) * 0.5;
  }

  /* Sum: '<S50>/Sum' incorporates:
   *  DiscreteIntegrator: '<S40>/Integrator'
   *  Inport: '<Root>/Kp'
   *  Product: '<S45>/PProd Out'
   */
  LLC_Control_CV_Y.fs = rtb_Integrator * LLC_Control_CV_U.Kp +
    LLC_Control_CV_DW.Integrator_DSTATE;

  /* Switch: '<S48>/Switch2' incorporates:
   *  Inport: '<Root>/Freq_low_limit'
   *  Inport: '<Root>/Freq_up_limit'
   *  RelationalOperator: '<S48>/LowerRelop1'
   *  RelationalOperator: '<S48>/UpperRelop'
   *  Switch: '<S48>/Switch'
   */
  if (LLC_Control_CV_Y.fs > LLC_Control_CV_U.Freq_up_limit) {
    /* Sum: '<S50>/Sum' incorporates:
     *  Outport: '<Root>/fs'
     */
    LLC_Control_CV_Y.fs = LLC_Control_CV_U.Freq_up_limit;
  } else if (LLC_Control_CV_Y.fs < LLC_Control_CV_U.Freq_low_limit) {
    /* Sum: '<S50>/Sum' incorporates:
     *  Inport: '<Root>/Freq_low_limit'
     *  Outport: '<Root>/fs'
     *  Switch: '<S48>/Switch'
     */
    LLC_Control_CV_Y.fs = LLC_Control_CV_U.Freq_low_limit;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Update for DiscreteIntegrator: '<S40>/Integrator' incorporates:
   *  Inport: '<Root>/Ki'
   *  Product: '<S37>/IProd Out'
   */
  LLC_Control_CV_DW.Integrator_IC_LOADING = 0U;
  LLC_Control_CV_DW.Integrator_DSTATE += rtb_Integrator * LLC_Control_CV_U.Ki *
    2.0E-5;

  /* End of Outputs for SubSystem: '<Root>/LLC_Control_CV' */
}

/* Model initialize function */
void LLC_Control_CV_initialize(void)
{
  /* SystemInitialize for Atomic SubSystem: '<Root>/LLC_Control_CV' */
  /* InitializeConditions for DiscreteIntegrator: '<S40>/Integrator' */
  LLC_Control_CV_DW.Integrator_IC_LOADING = 1U;

  LLC_Control_CV_U.V_ref=48;
  LLC_Control_CV_U.Freq_low_limit=160000;
  LLC_Control_CV_U.Freq_up_limit=240000;
  LLC_Control_CV_U.V_ref_low_limit=40;
  LLC_Control_CV_U.V_ref_up_limit=50;
  LLC_Control_CV_U.OBC_Temp_C=0;
  LLC_Control_CV_U.Thresholds.MaxCurrentLimit_A=50;
  LLC_Control_CV_U.Thresholds.OTWarningLimit_C=5;
  LLC_Control_CV_U.Thresholds.OTWarningLimit_C=10;
  LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_Out_V=30;
  LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_Out_V=25;
  LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_Out_V=70;
  LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_Out_V=75;
  LLC_Control_CV_U.Thresholds.UVWarningLimit_LLC_In_V=355;
  LLC_Control_CV_U.Thresholds.UVErrorLimit_LLC_In_V=350;
  LLC_Control_CV_U.Thresholds.OVWarningLimit_LLC_In_V=425;
  LLC_Control_CV_U.Thresholds.OVErrorLimit_LLC_In_V=430;
  LLC_Control_CV_U.Thresholds.OCWarningLimit_A=45;
  LLC_Control_CV_U.Thresholds.OCErrorLimit_A=50;
  LLC_Control_CV_U.Thresholds.VoltageProtectionTimeout_msec=30;
  LLC_Control_CV_U.Thresholds.TempProtectionTimeout_msec=30;
  LLC_Control_CV_U.Thresholds.CurrentProtectionTimeout_msec=30;
  


  /* End of SystemInitialize for SubSystem: '<Root>/LLC_Control_CV' */
}

/* Model terminate function */
void LLC_Control_CV_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
