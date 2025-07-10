%option noyywrap
%{

  #include "y.tab.h"
  #include <stdio.h>
  #include <stdint.h>
  #include <string.h>
  
  /* Defines ****************/
  #define YY_NEVER_INTERACTIVE 1  
  
  /* Variables **************/
  extern char retVal[];
  extern uint8_t aUI_UartRxBuffer[];
  
  /* Prototypes *************/
  void yyinit(void);
  int yylex(void);

  /* Help strings **********/
  const char help_msg[] = "\n\n****** DSMPS 3kW LLC User Interface ******\n\n\r- COMMAND FRAME:\n\r <COMMAND_ID> <STATE>\n\n\r- SET FRAME:\n\r <SET_PARAM_ID> <NUM_VAL>\n\n\r- GET FRAME:\n\r <GET_PARAM_ID>\n\n\r\
- HELP:\n\rhelp:\t\tframe type list\n\rhelp cmd:\tcommand frame list\n\rhelp set:\tset frame list\n\rhelp get:\tget frame list\n\rfwi:\t\tfw and UI version info\n\n\r";
  
  const char help_cmd_msg[] = "\n\n\r- COMMAND FRAME:\n\r <COMMAND_ID> <STATE>\n\n\r<COMMAND_ID>:\n\rout:\toutput\n\rsr:\tSynch. Rect.\n\rasr:\tAdaptive SR\n\rol:\topen loop\n\rbm:\tburst mode\n\rfan:\tcooling fan\n\n\r<STATE>:\n\ron\r\noff\n\n\r";
  
  const char help_set_msg[] = "\n\n\r- SET FRAME:\n\r <SET_PARAM_ID> <NUM_VAL>\n\n\r<SET_PARAM_ID>:\n\rkp:\tPI proportional gain\n\rki:\tPI integral gain\n\rfreq:\topen loop frequency [Hz]\n\rdead:\tdead time value [ns]\n\rdr1:\tdelay rising 1 [ns]\n\rdf1:\tdelay falling 1 [ns]\n\r\
dr2:\tdelay rising 2 [ns]\n\rdf2:\tdelay falling 2 [ns]\n\n\r";

const char help_get_msg[] = "\n\n\r- GET FRAME:\n\r <GET_PARAM_ID>\n\n\r<GET_PARAM_ID>:\n\rctr:\tcontrol parameters (Kp and Ki)\n\rmeas:\tmeasured values (Vout, Vin, Iout, Temp)\
\n\rflt:\tread and clear last fault\n\rconfig:\tconfiguration parameters\n\rpwm:\tPWM parameters (open loop frequency, dead time and SR delays)\n\n\r";

%}

%%
\n 
[\t ]+	/* ignore white spaces */

^fwi|info {sprintf(retVal, "\n- Info: FW V1.5, GUI V1.0 -\n\n\r");}
^help {sprintf(retVal, help_msg);}
^help[ ]?cmd {sprintf(retVal, help_cmd_msg);}
^help[ ]?set {sprintf(retVal, help_set_msg);}
^help[ ]?get {sprintf(retVal, help_get_msg);}

out|OUT|output {return TOKEN_OUT;}
SR|sr {return TOKEN_SR;}
ol|OL|(o|O)pen[ ]?(l|L)oop {return TOKEN_OL;}
asr|ASR|adaptive[ ]?(sr|SR) {return TOKEN_ADAPTIVE_SR;}
bm|BM|(b|B)urst[ ]?(m|M)ode {return TOKEN_BM;}
fan|FAN {return TOKEN_FAN;}

on|off yylval=!strcmp(yytext,"on"); return STATE;

kp|Kp {return TOKEN_KP_GAIN;}
ki|Ki {return TOKEN_KI_GAIN;}
freq|FREQ {return TOKEN_FREQ;}
dead|dead[ ]?time|dt|DT {return TOKEN_DEAD_TIME;}
(dr|DR)1 {return TOKEN_DR1;}
(dr|DR)2 {return TOKEN_DR2;}
(df|DF)1 {return TOKEN_DF1;}
(df|DF)2 {return TOKEN_DF2;}

ctr[l]?|CTR[L]? {return TOKEN_CTR_PARAM;}
meas|MEAS {return TOKEN_MEASURES;}
flt|FLT|fault|FAULT {return TOKEN_FAULT;}
cnf|CNF|config|CONFIG {return TOKEN_CONFIG_PARAM;}
pwm|PWM {return TOKEN_PWM_PARAM;}

[-+]?[0-9]+ {yylval=atoi(yytext); return NUMBER;}

[a-zA-Z]+ {return TOKEN_ERROR;}

%%

void yyinit(void)
{
    YY_BUFFER_STATE bp;

    bp = yy_scan_string((char*)aUI_UartRxBuffer);

    yy_switch_to_buffer(bp);

    yyparse();

    yy_delete_buffer(bp);
}
