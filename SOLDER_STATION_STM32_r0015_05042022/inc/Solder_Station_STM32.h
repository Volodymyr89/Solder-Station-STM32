#ifndef _Solder_Station_STM32_H
#define _Solder_Station_STM32_H
                  
#include "SSD1306.h"
#define Reed_Switch_Activated  ((GPIOA->IDR)&(GPIO_IDR_IDR7))

typedef enum {
 Turn_ON_Motor_FAN,
 Turn_OFF_Motor_FAN
} FAN_CNTRL_t;

typedef enum {
	NoClick,
	Click,
	Doubleclick,
}Monitor_Skip_Button_State_t;

typedef enum {Pressed, Released
} status_t;

typedef enum {STBY, SOLDER, SETTING_TEMPERATURE_SOLDER, FAN, SETTING_TEMPERATURE_FAN, FAN_Speed, SETTING_BARS_FAN, NO_SOLDER_DETECTED, NO_FAN_DETECTED,  Number_of_States} StateType;
											
typedef struct { StateType State; 
				 void (*func)(void);
			   } StateMachineType;

extern const uint8_t soldering_iron [];
extern const uint8_t FAN_Symb [];
extern const uint8_t Bracket_Right [];
extern const uint8_t Bracket_Left [];
extern const uint8_t Bar [];
extern const uint8_t Thermometer_symb [];
								
// functions prototypes
uint16_t SET_FAN_Motor_PWM_DutyCycle (void);
void Turn_OFF_FAN (void);
void ENCODER_init (void);
void REED_SWITCH_init (void);
void Show_SET_Solder_Data_to_Display(void);
void Show_SET_FAN_Data_to_Display(void);
//void Show_Current_Solder_Data_to_Display (void);
//void Show_Current_FAN_Data_to_Display (void);
void Draw_FAN_Symbol(void);
void Update_PID_Solder(void);
void Update_PID_FAN(void);
void Draw_FAN_Motor_Status_Bars (uint8_t y);
void Draw_FAN_Motor_Brackets(void);
void Data_Averager_For_Solder (void);
void PWM_Disable_SOLDER (void);
void Data_Averager_From_FAN(void);
void Skip_Button_init (void); // PA8
void SKIP_Button_Timer (void);
volatile  uint8_t Monitor_Skip_Button (bool Reset);
void PWM_Disable_Motor_FAN(void);
void PWM_Disable_FAN_HEATER(void);
void TIM1_init_Capture_Compare (void); // for PWM on PA9 
void FSM(void);
void SET_Solder_Data(void);
void Debug_UART_init(void);
void Debug_UARTSend( char *Bufer, uint32_t num);
void PB0_LED_main_STATUS(void);
void Show_SET_FAN_Bars_Data_to_Display(void);
void Draw_FAN_is_Cooling_map (uint8_t x);
void Make_Click_init(void);
void Make_Click(void);
// end of functions prototypes
#endif
