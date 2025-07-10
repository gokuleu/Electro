%{
  #include <stdio.h>
  #include <string.h>
  #include <stdint.h>
  #include "LLC_control_param.h"
  #include "LLC_Globals.h"
  #include "Fault_Processing.h"
  
  typedef struct 
 {  
   uint16_t hVout_int;
   uint16_t hVout_dec;
   uint16_t hVin_int;
   uint16_t hVin_dec;
   uint16_t hIout_int;
   uint16_t hIout_dec;
   uint16_t hTemp_int;
   uint16_t hTemp_dec;
  } returnMeasureStruct_t;
 
  int yywrap(void);
  int yyparse(void);
  void yyerror(const char *str);
  bool CheckOutOfRangeParam(int32_t hParameter, int32_t hLowerlimit, int32_t hUpperlimit);
  void updateConfigParam(void);
  void updateMeasureValues(void);
  void updateLastFaultOccurred(void);
  extern int yylex(void);
  
  char retVal[511];
  char configParam[] = {'d', 'd', 'd', 'd', 'd', 'd'};
  char sLastFaultOccurred[25];
  returnMeasureStruct_t returnMeasureStruct;

%}

%token NUMBER STATE TOKEN_ERROR
%token TOKEN_OUT TOKEN_SR TOKEN_OL TOKEN_ADAPTIVE_SR TOKEN_BM TOKEN_FAN
%token TOKEN_KP_GAIN TOKEN_KI_GAIN TOKEN_FREQ TOKEN_DEAD_TIME TOKEN_DR1 TOKEN_DR2 TOKEN_DF1 TOKEN_DF2
%token TOKEN_CTR_PARAM TOKEN_MEASURES TOKEN_FAULT TOKEN_CONFIG_PARAM TOKEN_PWM_PARAM

%%

frame: /* empty */
        | frame command
	| frame set
        | frame get
        ;

command:
        output_switch
	| synchrect_switch
        | openloop_switch
        | adaptive_sr_switch
        | burst_mode_switch
        | fan_switch
        ;

set:
	set_Kp_gain
	|set_Ki_gain
        |set_freq
        |set_dead_time
        |set_delay_r1
        |set_delay_r2
        |set_delay_f1
        |set_delay_f2
	;
        
get:
	get_control
        |get_measures
        |get_fault
        |get_config
        |get_pwm_param
        ;
          
        

output_switch:
	TOKEN_OUT STATE
	{
          if($2){
            DCDC_ConfigParamStruct.bConverterEnabled = TRUE;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal,"\n- Converter's output enabled -\n\n\r");
//            sprintf(retVal,"\n- debug output enabled 6 -\n\n\r");
          }
          else{
            DCDC_ConfigParamStruct.bConverterEnabled = FALSE;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal,"\n- Converter's output disabled -\n\n\r");
          }
	}
	;

openloop_switch:
          TOKEN_OL STATE
          {
            if($2){
              DCDC_ConfigParamStruct.bOpenLoopEnabled = TRUE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Open Loop Mode enabled -\n\n\r");
            }
            else{
              DCDC_ConfigParamStruct.bOpenLoopEnabled = FALSE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Open Loop Mode disabled -\n\n\r");
            }
          }
          ;
          
synchrect_switch:
	TOKEN_SR STATE
	{
          if($2){
            DCDC_ConfigParamStruct.bSREnabled = TRUE;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- Synchronous Rectification enabled -\n\n\r");
          }
          else{
            DCDC_ConfigParamStruct.bSREnabled = FALSE;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- Synchronous Rectification disabled -\n\n\r");
          }
	}
	;
       
                
adaptive_sr_switch:
          TOKEN_ADAPTIVE_SR STATE
          {
            if($2){
              DCDC_ConfigParamStruct.bAdaptiveSREnabled = TRUE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Adaptive SR enabled -\n\n\r");
            }
            else{
              DCDC_ConfigParamStruct.bAdaptiveSREnabled = FALSE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Adaptive SR disabled -\n\n\r");
            }
          }
          ;                
                
burst_mode_switch:
          TOKEN_BM STATE
          {
            if($2){
              DCDC_ConfigParamStruct.bBurstModeEnabled = TRUE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Burst Mode enabled -\n\n\r");
            }
            else{
              DCDC_ConfigParamStruct.bBurstModeEnabled = FALSE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Burst Mode disabled -\n\n\r");
            }
          }
          ;

fan_switch:
          TOKEN_FAN STATE
          {
            if($2){
              DCDC_ConfigParamStruct.bFanPWMDrivingEnabled = TRUE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Fan enabled -\n\n\r");
            }
            else{
              DCDC_ConfigParamStruct.bFanPWMDrivingEnabled = FALSE;
              DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
              sprintf(retVal, "\n- Fan disabled -\n\n\r");
            }
          }
          ;
                
set_Kp_gain:
	TOKEN_KP_GAIN NUMBER
	{
          if(CheckOutOfRangeParam($2, 0, S16_MAX) == TRUE){
            yyerror("parameter out of boundaries\n\r");
	    }
          else{
            DCDC_ConfigParamStruct.hRegulatorKpGain = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- Kp gain set to %d -\n\n\r", $2);
          }
	}
	;

set_Ki_gain:
	TOKEN_KI_GAIN NUMBER
	{
          if(CheckOutOfRangeParam($2, 0, S16_MAX) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hRegulatorKiGain = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- Ki gain set to %d -\n\n\r", $2);
          }
	}
	;

set_freq:
	TOKEN_FREQ NUMBER
	{
          if(CheckOutOfRangeParam($2, HRTIM_MIN_PWM_FREQ_HZ, HRTIM_MAX_PWM_FREQ_HZ) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.wOpenLoopFreq_Hz = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- Open Loop frequency set to %d Hz -\n\n\r", $2);
          }
	}
	;
        
set_dead_time:
	TOKEN_DEAD_TIME NUMBER
	{
          if(CheckOutOfRangeParam($2, DEAD_TIME_MIN_NS, DEAD_TIME_MAX_NS) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hDeadTimeFullBridge_ns = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- dead time set to %d ns -\n\n\r", $2);
          }
	}
	;        

set_delay_r1:
	TOKEN_DR1  NUMBER
	{
          if(CheckOutOfRangeParam($2, SYNCH_RECT_DELAY_RISING1_MIN_NS, SYNCH_RECT_DELAY_RISING1_MAX_NS) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hFixedSRDelayRising1_ns = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- delay rising 1 set %d ns -\n\n\r", $2);
          }
	}
	;
        
set_delay_r2:
	TOKEN_DR2  NUMBER
	{
          if(CheckOutOfRangeParam($2, SYNCH_RECT_DELAY_RISING2_MIN_NS, SYNCH_RECT_DELAY_RISING2_MAX_NS) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hFixedSRDelayRising2_ns = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- delay rising 2 set %d ns -\n\n\r", $2);
          }
	}
	;        

set_delay_f1:
	TOKEN_DF1  NUMBER
	{
          if(CheckOutOfRangeParam($2, SYNCH_RECT_DELAY_FALLING1_MIN_NS, SYNCH_RECT_DELAY_FALLING1_MAX_NS) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hFixedSRDelayFalling1_ns = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- delay falling 1 set %d ns -\n\n\r", $2);
          }
	}
	;
        
set_delay_f2:
	TOKEN_DF2  NUMBER
	{
          if(CheckOutOfRangeParam($2, SYNCH_RECT_DELAY_FALLING2_MIN_NS, SYNCH_RECT_DELAY_FALLING2_MAX_NS) == TRUE){
            yyerror("parameter out of boundaries\n\r");
          }
          else{
            DCDC_ConfigParamStruct.hFixedSRDelayFalling2_ns = $2;
            DCDC_ConfigParamStruct.bConfigurationChanged = TRUE;
            sprintf(retVal, "\n- delay falling 2 set %d ns -\n\n\r", $2);
          }
	}
	;

get_control:
        TOKEN_CTR_PARAM
        {
          sprintf(retVal, "\n- Kp = %d, Ki = %d\n\n\r", DCDC_ConfigParamStruct.hRegulatorKpGain, DCDC_ConfigParamStruct.hRegulatorKiGain);
        }
        ;

get_measures:
        TOKEN_MEASURES
        {          
          updateMeasureValues();          
          sprintf(retVal, "\n- Measures:\n\rVin = %d.%1d V, Vout = %d.%1d V\n\rIout = %d.%1d A, Temp = %d.%1d deg C\n\n\r", returnMeasureStruct.hVin_int, returnMeasureStruct.hVin_dec, returnMeasureStruct.hVout_int, returnMeasureStruct.hVout_dec, returnMeasureStruct.hIout_int, returnMeasureStruct.hIout_dec, returnMeasureStruct.hTemp_int, returnMeasureStruct.hTemp_dec);
        }
        ;

get_config:
        TOKEN_CONFIG_PARAM
        {
          updateConfigParam();
          sprintf(retVal, "\n- Configuration:\n\r\t\t%c\rOutput:\n\r\t\t%c\rOpen Loop Mode:\n\r\t\t%c\rSynch. Rect.:\n\r\t\t%c\rAdaptive SR:\n\r\t\t%c\rBurst Mode:\n\r\t\t%c\rFan:\n\n\r", configParam[0], configParam[1], configParam[2], configParam[3], configParam[4], configParam[5]);
        }
        ;
        
get_pwm_param:
        TOKEN_PWM_PARAM
        {
          sprintf(retVal, "\n- PWM parameters:\n\r\t\t\t%d Hz\rOpen loop freq.:\n\r\t\t\t%d ns\rDead time:\n\r\t\t\t%d ns\rDelay rising 1:\n\r\t\t\t%d ns\rDelay falling 1:\n\r\t\t\t%d ns\rDelay rising 2:\n\r\t\t\t%d ns\rDelay falling 2:\n\n\r",\
          DCDC_ConfigParamStruct.wOpenLoopFreq_Hz, DCDC_ConfigParamStruct.hDeadTimeFullBridge_ns, DCDC_ConfigParamStruct.hFixedSRDelayRising1_ns, DCDC_ConfigParamStruct.hFixedSRDelayFalling1_ns, DCDC_ConfigParamStruct.hFixedSRDelayRising2_ns, DCDC_ConfigParamStruct.hFixedSRDelayFalling2_ns);
        }
        ;

get_fault:
        TOKEN_FAULT
        {
          updateLastFaultOccurred();
          FLT_ClearAllSystemFaults();
          sprintf(retVal, "\n- Last fault occurred: %s\n\r- Fault cleared!\n\n\r", sLastFaultOccurred);
        }
        ;

%%

void yyerror(const char *str)
{
  sprintf(retVal, "\n- Error: %s\n\r", str);
}

int yywrap()
{
        return 1;
}

bool CheckOutOfRangeParam(int32_t hParameter, int32_t hLowerlimit, int32_t hUpperlimit){
  
  if((hParameter > hUpperlimit)||(hParameter < hLowerlimit))
  {
    return TRUE;
  }
  else{
    return FALSE;
  }
}

void updateConfigParam(void)
{
  configParam[0] = ((DCDC_ConfigParamStruct.bConverterEnabled == TRUE) ? 'e' : 'd');
  configParam[1] = ((DCDC_ConfigParamStruct.bOpenLoopEnabled == TRUE) ? 'e': 'd');
  configParam[2] = ((DCDC_ConfigParamStruct.bSREnabled == TRUE) ? 'e': 'd');
  configParam[3] = ((DCDC_ConfigParamStruct.bAdaptiveSREnabled == TRUE) ? 'e': 'd');
  configParam[4] = ((DCDC_ConfigParamStruct.bBurstModeEnabled == TRUE) ? 'e': 'd');
  configParam[5] = ((DCDC_ConfigParamStruct.bFanPWMDrivingEnabled == TRUE) ? 'e': 'd');
}

void updateMeasureValues(void)
{
  float fVout = OUT_VOLT_ADC_2_VOLT_VALUE(hVoutVoltageFiltered);
  float fVin = IN_VOLT_ADC_2_VOLT_VALUE(hVinVoltageFiltered);
  float fIout = HALL_IC_ADC_VALUE2AMP(DCDC_MeasureStruct.hIout);
  float fTemp = fTemperatureDeg;

  // debug values;
//  float fVout = 47.3f;
//  float fVin = 403.8f;
//  float fTemp = 38.3f;
//  float fIout = 54.7f;  
//  
  /* calc integer and decimal part of Vout */
  returnMeasureStruct.hVout_int = (uint16_t)fVout;
  returnMeasureStruct.hVout_dec = (uint16_t)((uint16_t)(fVout*10) - returnMeasureStruct.hVout_int*10);
    
  /* calc integer and decimal part of Vin */
  returnMeasureStruct.hVin_int = (uint16_t)fVin;
  returnMeasureStruct.hVin_dec = (uint16_t)((uint16_t)(fVin*10) - returnMeasureStruct.hVin_int*10);

  /* calc integer and decimal part of Iout */
  returnMeasureStruct.hIout_int = (uint16_t)fIout;
  returnMeasureStruct.hIout_dec = (uint16_t)((uint16_t)(fIout*10) - returnMeasureStruct.hIout_int*10);

  /* calc integer and decimal part of Temperature */
  returnMeasureStruct.hTemp_int = (uint16_t)fTemp;
  returnMeasureStruct.hTemp_dec = (uint16_t)((uint16_t)(fTemp*10) - returnMeasureStruct.hTemp_int*10);  

}

void updateLastFaultOccurred(void)
{
  uint16_t hLastShownFault = FLT_GetLastShownSystemFault();
  switch(hLastShownFault)
  {
    case DCDC_OUT_OVER_VOLT_ERROR: sprintf(sLastFaultOccurred, "Output overvoltage"); break;
    case DCDC_OUT_UNDER_VOLT_ERROR: sprintf(sLastFaultOccurred, "Output undervoltage"); break;
    case DCDC_IN_OVER_VOLT_ERROR: sprintf(sLastFaultOccurred, "Input overvoltage"); break;
    case DCDC_IN_UNDER_VOLT_ERROR: sprintf(sLastFaultOccurred, "Input undervoltage"); break;
    case DCDC_OVER_CURRENT_ERROR: sprintf(sLastFaultOccurred, "Transformer overcurrent"); break;
    case DCDC_OUT_OVER_CURRENT_ERROR: sprintf(sLastFaultOccurred, "Output overcurrent"); break;
    case DCDC_OVER_TEMP_ERROR: sprintf(sLastFaultOccurred, "Heatsink overtemperature"); break;
    case DCDC_STARTUP_FAILED_ERROR: sprintf(sLastFaultOccurred, "Start-up failed"); break;
    
    case DCDC_PRIMARY_SIDE_ERROR: sprintf(sLastFaultOccurred, "PFC error"); break;
    case DCDC_COMMUNICATION_ERROR: sprintf(sLastFaultOccurred, "Communication error"); break;    
    
    case DCDC_NO_ERROR:
    default: sprintf(sLastFaultOccurred,"No Error"); break;  
  }  
}
 
