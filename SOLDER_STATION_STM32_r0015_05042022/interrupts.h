#ifndef _INTERRUPTS_H
#define _INTERRUPTS_H
#include "stm32f10x.h"                  // Device header
#include "stdbool.h" 
#define STATUS_ENABLED 0xFF
#define STATUS_DISABLED 0x01


typedef struct { volatile uint8_t Show_SET_Solder_Data_to_Display_READY, 
		                     Show_SET_FAN_Data_to_Display_READY, 
                         RUN_Update_PID_Solder, 
	                       RUN_Update_PID_FAN,
                         RUN_Update_Duty_Cycle_Bars;
												}Updates_t;
typedef enum { FAN_Activated, FAN_Deactivated, CoolDown_and_OFF_FAN
} FAN_Status;

typedef enum {SOLDER_Activated, SOLDER_Deactivated
} SOLDER_Status;
#endif