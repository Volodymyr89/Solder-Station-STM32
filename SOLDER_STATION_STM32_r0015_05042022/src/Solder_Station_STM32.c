#include "Solder_Station_STM32.h"  
#include "stm32f10x.h"     
#include "STM32_GPIO_lib.h"
#include "Short_Delays_us_ms.h"
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "SPI_Transmitt.h"
#include "SSD1306.h"
#include "interrupts.h"
#include "stdio.h" 
#include "math.h" 
#include "string.h"

// variables prototypes
uint16_t SETed_Solder_Data=200, SETed_FAN_Data = 100;
float  Current_Temp_Solder = 0.0, Current_Temp_FAN = 0.0;
uint16_t Bit_Map_Size = 0;
uint8_t Bars_num = 4;
extern StateType State;
volatile status_t Skip_Button_Status = Released;
extern FAN_Status FAN_Status_t;
extern volatile Updates_t Status_Updates_From_Interrupt_t;
// end of variables prototypes 

// bit map for solder
const uint8_t soldering_iron [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x30, 0x00, 
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x00, 
0x00, 0x07, 0x00, 0x00, 0x00, 0x4e, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 
0x00, 0xf8, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x07, 0xc0, 0x08, 0x00, 
0x1f, 0x80, 0x18, 0x00, 0x1f, 0x00, 0x10, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x1e, 0x00, 0x40, 0x00, 
0x20, 0xe0, 0x80, 0x00, 0x21, 0x11, 0x00, 0x00, 0x42, 0x0e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00
};
// bit map for FAN
const uint8_t FAN_Symb [] = {
0x01, 0xfe, 0x00, 0x07, 0x03, 0xc0, 0x18, 0xc0, 0x60, 0x33, 0xf0, 0x30, 0x63, 0xf0, 0x18, 0x41, 
0xf8, 0x0c, 0xc0, 0x60, 0x0c, 0x80, 0x31, 0xe4, 0x80, 0x3b, 0xe4, 0x83, 0x87, 0xe4, 0xc7, 0xc7, 
0xec, 0x47, 0xc0, 0x08, 0x67, 0xc0, 0x18, 0x33, 0x80, 0x30, 0x1c, 0x00, 0xe0, 0x07, 0x03, 0x80, 
0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xff, 0x00, 0x07, 0xff, 0x80, 0x0f, 
0xff, 0xc0
};
// bit map for brackets
const uint8_t Bracket_Right [] = {
0xFC, 
0xFC, 
0xFC, 
0x07, 
0x07, 
0x07, 
0x07, 
0x07,  
0x07,  
0x07, 
0x07, 
0x07, 
0xFC, 
0xFC, 
0xFC, 
};

const uint8_t Bracket_Left [] = {
0x1F, 
0x1F, 
0x1F, 
0xE0, 
0xE0, 
0xE0, 
0xE0, 
0xE0,  
0xE0,  
0xE0, 
0xE0, 
0xE0, 
0x1F, 
0x1F, 
0x1F, 
};
// end of bit map for brackets

// bit map for FAN speed bars 
const uint8_t Bar [] = {
0xFF, 
0xFF, 
0xFF,
};
// bit map for FAN durin cooling
const uint8_t FAN_is_Cooling [] = {
0x01,
0x04,
0x08,
0x10,
0x20,
0x40,
0x40,
0x20, 
0x08, 
0x02,
0x01,
0x01,
0x02,
0x08,
0x20,
0x40,
0x80,
0x80,
};
// make click during encoder operation
void Make_Click_init(void){
	GPIO_EN (GPIO_PINB,  PIN9_CRH, Output_Push_Pull_2MHz); 
	GPIO_EN (GPIO_PINB,  PIN11_CRH, Output_Push_Pull_2MHz);
	GPIOB-> ODR |= GPIO_ODR_ODR9;
	GPIOB-> ODR &= ~GPIO_ODR_ODR11;
}
void Make_Click(void){
	uint16_t delay = 1000;
	static bool flip = true;
	if(flip){
			flip ^= flip;
			GPIOB-> ODR ^= GPIO_ODR_ODR9;	
			while(delay--){}
			GPIOB-> ODR ^= GPIO_ODR_ODR11;
		}
	 else{
		  flip ^= flip;
			GPIOB-> ODR ^= GPIO_ODR_ODR11;
			while(delay--){}
			GPIOB-> ODR ^= GPIO_ODR_ODR9;
	 }		 
}
// end of make click during encoder operation

// draw FAN is cooling function
void Draw_Fan_is_Cooling(void)
{	
	 for(uint8_t i = 0; i < 3; i++)
	 {
			Draw_FAN_is_Cooling_map(7 + i*6);
	 }
}
	void ShowFanisCooling(void){
	if (FAN_Status_t == CoolDown_and_OFF_FAN){
		volatile static uint8_t visiblecount = 0;
			if(visiblecount < 2){
				Draw_Fan_is_Cooling();
			}
	 else if (visiblecount >= 3)  {
				visiblecount = 0;
	 }
	 visiblecount++;
	}
}
// draw stan by state 
 void Draw_STBY_Symbol (void)
 {
	 Bit_Map_Size = (&Bracket_Left)[1] - Bracket_Left;
	 SSD1306_Clear_Display(Display_Buffer());
	 Adafruit_GFX_drawBitmap (3, 0, Bracket_Left, Bit_Map_Size);
	 Adafruit_GFX_drawBitmap (115, 0, Bracket_Right, Bit_Map_Size);
	 Adafruit_GFX_setFont(&Alison_finch10pt7b);	
	 Set_Cursor (28,0);
	 ssd1306_WriteString ("STBY MODE");
	 Set_Cursor (36,30);
	 Adafruit_GFX_setFont(&imelda16pt7b);
	 ssd1306_WriteString ("200 C");
	 Set_Cursor (85,27);
	 Adafruit_GFX_drawCircle(2);
	 while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}
 }
// draw solder symbol 
 void Draw_SOLDER_Symbol(void)
{
	while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Adafruit_GFX_drawBitmap (10, 0, Bracket_Left, Bit_Map_Size);
	Adafruit_GFX_drawBitmap (100, 0, Bracket_Right, Bit_Map_Size);
	Set_Cursor(35,0);
	Adafruit_GFX_setFont(&Alison_finch10pt7b);
	ssd1306_WriteString("SOLDER");
}

/*
void Draw_NO_Solder_detected(void){
   while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Set_Cursor(5,0);
	Adafruit_GFX_setFont(&Alison_finch10pt7b);
	ssd1306_WriteString("!!!NO SOLDER!!!");
	while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}
}

void Draw_NO_FAN_detected(void){
   while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Set_Cursor(5,15);
	Adafruit_GFX_setFont(&Alison_finch10pt7b);
	ssd1306_WriteString("!!!NO FAN!!!");
}
*/

// draw FAN symbol
void Draw_FAN_Symbol(void)
{
	Bit_Map_Size = (&Bracket_Left)[1] - Bracket_Left;
	 while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){} // wait until dispaly is cleared
	Adafruit_GFX_drawBitmap (3, 0, Bracket_Left, Bit_Map_Size);
	Adafruit_GFX_drawBitmap (115, 0, Bracket_Right, Bit_Map_Size);
	Set_Cursor(24,0);
	Adafruit_GFX_setFont(&Alison_finch10pt7b);
	ssd1306_WriteString("THERMO FAN");
}
// draw FAN speed symbol 
void Draw_FAN_Speed_Symbol(void)
{
	 Bit_Map_Size = (&Bracket_Left)[1] - Bracket_Left;
	 Adafruit_GFX_drawBitmap (3, 0, Bracket_Left, Bit_Map_Size);
	 Adafruit_GFX_drawBitmap (115, 0, Bracket_Right, Bit_Map_Size);
	 Set_Cursor(24,0);
 	Adafruit_GFX_setFont(&Alison_finch10pt7b);
	ssd1306_WriteString("FAN SPEED");
}

// PWM for solder
void TIM2_init_Capture_Compare (void) 
{
	GPIO_EN (GPIO_PINA, PIN1_CRL, AF_Alternate_Function_Push_Pull_50MHz); //PA1 as AF for Solder channel 2 
	TIM2->CR1 &= ~TIM_CR1_CEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 8;	// 8MHz may be changed to 12 MHz
	TIM2->ARR= 32000; //250 Hz
	TIM2->DIER |= TIM_DIER_CC2IE;// Capture/Compare 2 interrupt enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  // Channel 2 PA1 PWM
}
// enable PWM for solder
void PWM_EN_SOLDER (void)
{
	TIM2->CCER |= TIM_CCER_CC2E;  //Capture compare enable
	TIM2->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ (TIM2_IRQn);
}
// disable PWM for solder
void PWM_Disable_SOLDER (void)
{
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_2 & TIM_CCMR1_OC2M_1);
	TIM2->CR1 &= ~TIM_CR1_CEN;
	NVIC_DisableIRQ (TIM2_IRQn);
}
// PWM for FAN
void TIM1_init_Capture_Compare (void)
{
	static bool TIM1enabled = false;
	if(!TIM1enabled){
		TIM1enabled = true;
		GPIO_EN (GPIO_PINA, PIN9_CRH, AF_Alternate_Function_Push_Pull_50MHz); //PA9 as AF for FAN motor channel TIM1_CH_2
		GPIO_EN (GPIO_PINA, PIN10_CRH, AF_Alternate_Function_Push_Pull_50MHz); //PA10 as AF for FAN channel TIM1_CH_3
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;	
		TIM1->PSC = 199;	// 800 Hz
		TIM1->ARR= 32000; // 25 Hz
		TIM1->DIER |= TIM_DIER_CC3IE;// Capture/Compare 3 interrupt enable
	
///////////////////////////////////// _FAN MOTOR CONTROL PART_ ////////////////////////////////////////////////////////////
		TIM1->CCR2 = 20000; // just init
		TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM mode for 3 Channel to control motor of FAN blower
///////////////////////////////////// FAN HEATER CONTROL PART_ ////////////////////////////////////////////////////////////
		TIM1->CCR3 = 10000; // just init
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode for 3 Channel
		TIM1->BDTR |= TIM_BDTR_MOE;
	}		
}
// enable PWM for FAN
void PWM_EN_FAN_HEATER (void)
{
	  TIM1->CCER |= TIM_CCER_CC3E;  //Capture compare output enable
		NVIC_EnableIRQ (TIM1_CC_IRQn);
}
// disable PWM for FAN
void PWM_Disable_FAN_HEATER (void)
{
	TIM1->CCER &= ~TIM_CCER_CC3E;  //Capture compare output disable
  //	NVIC_DisableIRQ (TIM1_CC_IRQn);
}
// enable PWM for FAN motor 
void PWM_EN_OFF_Motor_FAN(FAN_CNTRL_t FAN_CONTROL_STATUS)
{
	if(FAN_CONTROL_STATUS == Turn_ON_Motor_FAN){
		TIM1->CCER |= TIM_CCER_CC2E;  //Capture compare output enable
		TIM1->BDTR |= TIM_BDTR_MOE;
		TIM1->CR1 |= TIM_CR1_CEN;
	}
 	else if (FAN_CONTROL_STATUS == Turn_OFF_Motor_FAN){
		TIM1->CR1 &= ~TIM_CR1_CEN;
		TIM1->CCER &= ~TIM_CCER_CC2E;  //Capture compare output disable
	}
}


// skip button timer init
void SKIP_Button_Timer (void)
{
	static uint8_t TIMER3_SKIP_Button_init = 0x00;
	
if (TIMER3_SKIP_Button_init==0x00)
{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		TIM3->PSC = 719;
		TIM3->EGR |= TIM_EGR_UG;  // to have divider updated
		TIM3->SR &= ~TIM_SR_UIF; // clear interrupt flag
		TIM3->DIER |= TIM_DIER_UIE;
		TIM3->ARR = 30000; // 100 ms
		TIM3->CR1 |= TIM_CR1_CEN;
		TIM3->CNT = 0;
		NVIC_EnableIRQ (TIM3_IRQn);	
		TIMER3_SKIP_Button_init=0xFF;
	}
	
	else
	{
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->ARR = 30000; // 100 ms
		TIM3->CR1 |= TIM_CR1_CEN;
	}
}
// PID section
typedef struct PID{
	float	Kp;
	float Ki;
	float Kd;
	float Error;
	float Derivative;
	float Last_Error;
	int16_t PWM_Duty_Cycle;
	uint16_t PWM_Duty_Cycle_ready;
	float Target_Temp;
}PID_t;

PID_t PID;

uint16_t PID_Solder (PID_t *PID_Solder){ 
	uint16_t PWM_Duty_Cycle_ready = 0;
	static double integral = 0.0;
	float Prop = 0.0;
	float Int = 0.0;
	float Diff = 0.0;
	PID_Solder->Kp = 220.0;
	PID_Solder->Ki = 0.1;
	PID_Solder->Kd = 0.3;
	PID_Solder->Target_Temp = (float)SETed_Solder_Data; // thats what encoder gives
	Current_Temp_Solder = SPI1_Receive_Temperature(SOLDER_Chose);
	PID_Solder->Error = PID_Solder->Target_Temp - Current_Temp_Solder;
	integral += PID_Solder->Error;
	PID_Solder->Derivative = PID_Solder->Error - PID_Solder->Last_Error;
	
	Prop = PID_Solder->Kp * PID_Solder->Error;
	if(Prop >= 32000)
		Prop = 32000;
	Int = (float)(PID_Solder->Ki * integral);
	Diff = PID_Solder->Kd * PID_Solder->Derivative;
	PID_Solder->Last_Error = PID_Solder->Error;
	
	PID_Solder->PWM_Duty_Cycle = (int16_t)round((Prop + Int + Diff));
	
	if (PID_Solder->PWM_Duty_Cycle >= 32000)
	{
		PWM_Duty_Cycle_ready = 32000;
	}
	else if (PID_Solder-> PWM_Duty_Cycle <= 0)
	{
		PWM_Duty_Cycle_ready = 0;
	}
	else 
	{
		PWM_Duty_Cycle_ready = (uint16_t)PID_Solder -> PWM_Duty_Cycle; // to debug can be setted constant digit
	}

	return PWM_Duty_Cycle_ready;	
}

uint16_t PID_FAN (PID_t *PID_FAN){ 
	uint16_t PWM_Duty_Cycle_ready = 0;
	static double integral = 0.0;
	float Prop = 0.0;
	float Int = 0.0;
	float Diff = 0.0;
	PID_FAN->Kp = 330.0;
	PID_FAN->Ki = 0.9;
	PID_FAN->Kd = 0.7;
	PID_FAN->Target_Temp = (float)SETed_FAN_Data; // thats what encoder gives
	Current_Temp_FAN = SPI1_Receive_Temperature(FAN_Chose);
		
	PID_FAN->Error = (PID_FAN->Target_Temp) - Current_Temp_FAN;
	integral += PID_FAN->Error;
	PID_FAN->Derivative = PID_FAN->Error - PID_FAN->Last_Error;
	
	Prop = PID_FAN->Kp * PID_FAN->Error;
	if(Prop >= 32000)
		Prop = 32000;
	Int = (float)(PID_FAN->Ki * integral);
	Diff = PID_FAN->Kd * PID_FAN->Derivative;
	PID_FAN->Last_Error = PID_FAN->Error;
	
	PID_FAN->PWM_Duty_Cycle = (int16_t)round((Prop + Int + Diff));
	
	if (PID_FAN->PWM_Duty_Cycle >= 32000)
	{
		PWM_Duty_Cycle_ready = 32000;
	}
	else if (PID_FAN-> PWM_Duty_Cycle <= 0)
	{
		PWM_Duty_Cycle_ready = 0;
	}
	else 
	{
		PWM_Duty_Cycle_ready = (uint16_t)PID_FAN -> PWM_Duty_Cycle; // to debug can be setted constant digit
	}

	return PWM_Duty_Cycle_ready;	
}
// end of PID section
void Draw_Bars(uint8_t num){
 uint8_t s = 0;
	for (uint8_t i =0; i < num; i++){
		Draw_FAN_Motor_Status_Bars(55 - s);
		s += 5;
	}
}

// set temperature for solder
void SET_Solder_Data(void)
{
		if ((SETed_Solder_Data>450) || (SETed_Solder_Data<200))
		{
			if(SETed_Solder_Data>450)
				SETed_Solder_Data=450;
			else if(SETed_Solder_Data<200)
				SETed_Solder_Data=200;
		}
		else{		
			if ((EXTI->PR)&(EXTI_PR_PR6))
		{
				if ((GPIOB->IDR&GPIO_IDR_IDR6)&&(!(GPIOB->IDR&GPIO_IDR_IDR7))){
						SETed_Solder_Data -= 5;
				}
			}
		
		else if ((EXTI->PR)&(EXTI_PR_PR7))
			{
					if ((GPIOB->IDR&GPIO_IDR_IDR7)&&(!(GPIOB->IDR&GPIO_IDR_IDR6))){
							SETed_Solder_Data += 5;
					}
			}
		}
		EXTI->PR |= EXTI_PR_PR6 | EXTI_PR_PR7;
		if(Status_Updates_From_Interrupt_t.Show_SET_Solder_Data_to_Display_READY == STATUS_DISABLED)
				Status_Updates_From_Interrupt_t.Show_SET_Solder_Data_to_Display_READY = STATUS_ENABLED;
}
// set temperature for FAN
void SET_FAN_Data(void)
{
	if ((SETed_FAN_Data>400) || (SETed_FAN_Data<100))
		{
			if(SETed_FAN_Data>400)
				SETed_FAN_Data=400;
			else if(SETed_FAN_Data<100)
				SETed_FAN_Data=100;
		}
		else{		
			if ((EXTI->PR)&(EXTI_PR_PR6))
		{
				if ((GPIOB->IDR&GPIO_IDR_IDR6)&&(!(GPIOB->IDR&GPIO_IDR_IDR7))){
						SETed_FAN_Data -= 10;
				}
			}
		
		else if ((EXTI->PR)&(EXTI_PR_PR7))
			{
					if ((GPIOB->IDR&GPIO_IDR_IDR7)&&(!(GPIOB->IDR&GPIO_IDR_IDR6))){
							SETed_FAN_Data += 10;
					}
			}
		}
		EXTI->PR |= EXTI_PR_PR6 | EXTI_PR_PR7;
		if(Status_Updates_From_Interrupt_t.Show_SET_FAN_Data_to_Display_READY == STATUS_DISABLED)
				Status_Updates_From_Interrupt_t.Show_SET_FAN_Data_to_Display_READY = STATUS_ENABLED;
	
}
// set speed for FAN motor
void SET_FAN_Motor_PWM_DutyCycle_and_BARs (void)
{	
	static uint16_t FAN_PWM_Data = 20000;
			if ((EXTI->PR)&(EXTI_PR_PR6))
		{
				EXTI->PR |= EXTI_PR_PR6;
				if ((GPIOB->IDR&GPIO_IDR_IDR6)&&(!(GPIOB->IDR&GPIO_IDR_IDR7)) && (FAN_PWM_Data >= 15000) && (Bars_num > 2)){
						FAN_PWM_Data -= 3000;
						Bars_num--;
				}
			}
		
		else if ((EXTI->PR)&(EXTI_PR_PR7))
			{
					EXTI->PR |= EXTI_PR_PR7;
					if ((GPIOB->IDR&GPIO_IDR_IDR7)&&(!(GPIOB->IDR&GPIO_IDR_IDR6)) && (FAN_PWM_Data <= 32000) && (Bars_num < 8)){
						FAN_PWM_Data += 3000;
				  	Bars_num++;
					}
			}	
	  TIM1->CCR2 = FAN_PWM_Data;
		Status_Updates_From_Interrupt_t.RUN_Update_Duty_Cycle_Bars = STATUS_ENABLED;
		
}
// show speed of motor to display
void Show_SET_FAN_Bars_Data_to_Display(void){
	while ( Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Draw_Bars(Bars_num);
	Draw_FAN_Speed_Symbol();
	while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}
}
//show seted solder data to OLED
void Show_SET_Solder_Data_to_Display(void)
{
	char Solder_Data_tostring[5];
	sprintf(Solder_Data_tostring, "%u", SETed_Solder_Data);
	while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Draw_SOLDER_Symbol();
	Adafruit_GFX_setFont(&imelda16pt7b); 
	Set_Cursor(29,33);
	ssd1306_WriteString(Solder_Data_tostring);
	Set_Cursor(81,35);
	Adafruit_GFX_drawChar ('C', Display_Buffer());
	Set_Cursor (80,28);
	Adafruit_GFX_drawCircle(2);
	while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}
}
//show seted FAN data to OLED
void Show_SET_FAN_Data_to_Display(void)
{
	char Solder_FAN_tostring[5];
	sprintf(Solder_FAN_tostring, "%u", SETed_FAN_Data);
	while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
	Draw_FAN_Symbol();
	Adafruit_GFX_setFont(&imelda16pt7b);
	Set_Cursor(29,33);
	ssd1306_WriteString(Solder_FAN_tostring);
	Set_Cursor(81,35);
	Adafruit_GFX_drawChar ('C', Display_Buffer());
	Set_Cursor (80,28);
	Adafruit_GFX_drawCircle(2);
	while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}

}
//show averaged solder data to OLED
void Data_Averager_For_Solder (void)
{
	volatile static uint8_t i=0;
  volatile static	double Collect_Samples =0.0;
	volatile float Average =0.0;
	volatile uint16_t Value_transfer =0;
	char Dynamic_Solder_Data_tostring[5];
	
	if (i < 50)
	{
		Collect_Samples += Current_Temp_Solder;
		i++;
	}
	
	else
	{	
  	Average = (float)(Collect_Samples/50);
		Collect_Samples =0;
		i = 0;
	    Value_transfer = (uint16_t)round(Average);
	    sprintf(Dynamic_Solder_Data_tostring, "%u", Value_transfer);
	    Draw_SOLDER_Symbol();
			ShowFanisCooling();
	    Adafruit_GFX_setFont(&imelda16pt7b); 
	    Set_Cursor(29,33);
	    ssd1306_WriteString(Dynamic_Solder_Data_tostring);
	    Set_Cursor(81,35);
	    Adafruit_GFX_drawChar ('C', Display_Buffer());
	    Set_Cursor (80,28);
	    Adafruit_GFX_drawCircle(2);
	    while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}	
	}
}
//show averaged FAN data to OLED
 void Data_Averager_From_FAN (void)
{
	 static uint8_t i=0;
   static double Collect_Samples =0.0;
	 float Average =0.0;
	 uint16_t Value_transfer =0;
	char Dynamic_FAN_Data_tostring[5];
	
	if (i < 12)
	{
		Collect_Samples += Current_Temp_FAN;
		i++;
	}
	
	else
	{	
  	Average = (float)(Collect_Samples/12);
		Collect_Samples =0;
		i = 0;
	    Value_transfer = (uint16_t)round(Average);
	    sprintf(Dynamic_FAN_Data_tostring, "%u", Value_transfer);
	    Draw_FAN_Symbol();
	    Adafruit_GFX_setFont(&imelda16pt7b); 
	    Set_Cursor(29,33);
	    ssd1306_WriteString(Dynamic_FAN_Data_tostring);
	    Set_Cursor(81,35);
	    Adafruit_GFX_drawChar ('C', Display_Buffer());
	    Set_Cursor (80,28);
	    Adafruit_GFX_drawCircle(2);
	    while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}	
	}
}

// update PID with new info
void Update_PID_Solder(void)
{
		TIM2->CCR2 = PID_Solder(&PID);
}


void Update_PID_FAN(void)
{
	 TIM1->CCR3 = PID_FAN(&PID);
}
// end of update PID with new info

// init ext. interrupts for encoder
void Ext_interrupt_to_PINB6_PINB7_init (void)
{
	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN; // must be here for normal EXTI work
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB | AFIO_EXTICR2_EXTI7_PB; //  AFIO->EXTICR[1] starts from 0 in datasheet from 1
	EXTI->RTSR |= EXTI_RTSR_TR6 | EXTI_RTSR_TR7;
	
	EXTI->PR |= EXTI_PR_PR6; 
	EXTI->PR |= EXTI_PR_PR7;
	
  EXTI->IMR |= EXTI_IMR_MR6 | EXTI_IMR_MR7;
	NVIC_EnableIRQ (EXTI9_5_IRQn);

}

// init encoder
void ENCODER_init (void)
{	
	GPIO_EN (GPIO_PINB,  PIN6_CRL, Input_Pull_Down); // input for Encoder  
	GPIO_EN (GPIO_PINB,  PIN7_CRL, Input_Pull_Down); // input for Encoder
	Ext_interrupt_to_PINB6_PINB7_init ();    
}
// init skip button
void Skip_Button_init (void) // PA8
{
	GPIO_EN (GPIO_PINA, PIN8_CRH, Input_Pull_Up); 
}

// monitor skip button was pressed
volatile  uint8_t Monitor_Skip_Button (bool Reset)
{
	static uint8_t Skip_Button_CNT =0;
	if (Reset == true){
		Skip_Button_CNT = 0;}
	
	 if ((!(GPIOA->IDR & GPIO_IDR_IDR8)) && (Skip_Button_Status == Released))
	{
		Skip_Button_Status = Pressed;
		Skip_Button_CNT++;
		Make_Click();
		SKIP_Button_Timer();
	}
	else if (((GPIOA->IDR & GPIO_IDR_IDR8))&&(Skip_Button_Status == Pressed))
	{
		Skip_Button_Status = Released;
	}
		return (Skip_Button_CNT);
}
// skip button timer
void DLY4_Delay_ms_INTERRUPT (uint16_t Delay)
{
 static	uint8_t DLY4_init = 0x00;
	
	if (DLY4_init==0x00)
	{
		DLY4_init = 0xFF;
	  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	  TIM4->PSC = 3599; // 3600
	  TIM4->EGR |= TIM_EGR_UG;  // to have divider updated
	  TIM4->SR &= ~TIM_SR_UIF; // clear interrupt flag
	  TIM4->DIER |= TIM_DIER_UIE;
	  TIM4->ARR = Delay*10;
	  TIM4->CR1 |= TIM_CR1_CEN;
	  TIM4->CNT = 0;
	  NVIC_EnableIRQ (TIM4_IRQn);
	}	
	
	else
{
	TIM4->CR1 &= ~TIM_CR1_CEN;
	TIM4->EGR |= TIM_EGR_UG;  // to have divider updated
	TIM4->SR &= ~TIM_SR_UIF; // clear interrupt flag
	TIM4->ARR = Delay*10;
	TIM4->CR1 |= TIM_CR1_CEN;
}

}
// wait until FAN is cooled down and turn OFF it
void Turn_OFF_FAN (void){
	
	if (FAN_Status_t == CoolDown_and_OFF_FAN){
		uint16_t Received_Temp = 0;
		static uint8_t CNT = 5;
		PWM_Disable_FAN_HEATER();
		if (CNT <= 1)
		{
			Received_Temp = SPI1_Receive_Temperature(FAN_Chose); 
			if(Received_Temp <= 60){
				FAN_Status_t = FAN_Deactivated;
				PWM_EN_OFF_Motor_FAN(Turn_OFF_Motor_FAN);
			}
			CNT = 5;
		}
		CNT--;
	}
}

// STBY mode
void STBY_Mode_FSM (void)
{		
	  #ifdef DEBUG
		Debug_UARTSend("********************Entered STBY State**************************************\r\n");
   	#endif	
		SETed_Solder_Data=200, SETed_FAN_Data = 100;
		while ( Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
		Draw_STBY_Symbol();
		TIM2_init_Capture_Compare (); // for PWM
		PWM_EN_SOLDER();	// turn on Solder PWM
}
/*
void NO_Solder_detected_FSM (void){
 PWM_Disable_SOLDER();
 Draw_NO_Solder_detected();
}

void NO_FAN_detected_FSM (void){
// PWM_Disable_FAN_HEATER();
  //PWM_EN_OFF_Motor_FAN(Turn_OFF_Motor_FAN);
 //Draw_NO_FAN_detected();  
}
*/
// set temperature solder
 void SETTING_TEMPERATURE_SOLDER_FSM (void)
 {
		SET_Solder_Data();
	  DLY4_Delay_ms_INTERRUPT(1000); // after 2s jump to SOLDER
 }
// set temperature FAN
 void SETTING_TEMPERATURE_FAN_FSM (void)
 {
	 #ifdef DEBUG
	 Debug_UARTSend("Entered SETTING_TEMPERATURE_FAN_FSM State\n");
	 #endif
	 SET_FAN_Data();
	 DLY4_Delay_ms_INTERRUPT(1000); // after 2s jump to FAN
 }
// solder FSM state
void SOLDER_FSM (void)
{
    #ifdef DEBUG
		Debug_UARTSend("Entered SOLDER_FSM State\n\r");
		#endif
		while (Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
		Draw_SOLDER_Symbol();
}
// FAN FSM state
void FAN_FSM (void)
{
	#ifdef DEBUG
  Debug_UARTSend("Entered FAN_FSM State\n\r");
  #endif
	TIM1_init_Capture_Compare ();
	FAN_Status_t = FAN_Activated;
	PWM_EN_OFF_Motor_FAN(Turn_ON_Motor_FAN);
	SSD1306_Clear_Display(Display_Buffer());
	Draw_FAN_Symbol();
	PWM_EN_FAN_HEATER();	// turn on FAN PWM Heater
}
// FAN speed FSM state
void FAN_Speed_FSM(void)
{
	 while ( Clear_Compleated != SSD1306_Clear_Display(Display_Buffer())){}
   Draw_FAN_Speed_Symbol();
	 Draw_Bars(Bars_num);
	 while (Drawing_Compleated != SSD1306_Draw_Display(Display_Buffer())){}
}
// setting bars to display FSM state
void SETTING_BARS_FAN_FSM (void){
	
 SET_FAN_Motor_PWM_DutyCycle_and_BARs();
	
}
// draw motor speed bars
void Draw_FAN_Motor_Status_Bars (uint8_t y)
{
	uint8_t s =0;
 	static uint16_t Bit_Size = (&Bar)[1] - Bar;
	
	for(uint8_t i = 0; i < 7; i++){
		Adafruit_GFX_drawBitmap(35 + s, y, Bar, Bit_Size);
		s += 8;
	}
}
// draw FAN is cooling symbol
void Draw_FAN_is_Cooling_map (uint8_t x)
{
 	static uint16_t Bit_Size = (&FAN_is_Cooling)[1] - FAN_is_Cooling;
		Adafruit_GFX_drawBitmap(x, 20, FAN_is_Cooling, Bit_Size);
}
// debug UART initialization
void Debug_UART_init(void){
	
	/* Enable USART1 and GPIOA clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_EN (GPIO_PINA, PIN2_CRL, AF_Alternate_Function_Push_Pull_10MHz);
	
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	//Enable USART2
	USART_Cmd(USART2, ENABLE);
	
}
// send data via UART
void Debug_UARTSend( char *Bufer, uint32_t num)
{
	char buf[30];
	volatile int i=0;
	strcpy(buf, Bufer);
	sprintf(buf, " %d ", num);
    while (buf[i]!='\0')
    {
        USART_SendData(USART2, buf[i]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
        }
				i++;
    }
}
// state machine enum
StateMachineType StateMachine[9] = 
{
	{STBY, STBY_Mode_FSM},
	{SOLDER, SOLDER_FSM},
	{SETTING_TEMPERATURE_SOLDER,  SETTING_TEMPERATURE_SOLDER_FSM},
	{FAN, FAN_FSM},
	{SETTING_TEMPERATURE_FAN, SETTING_TEMPERATURE_FAN_FSM},
	{FAN_Speed, FAN_Speed_FSM},
	{SETTING_BARS_FAN, SETTING_BARS_FAN_FSM},
	{NO_SOLDER_DETECTED, NO_Solder_detected_FSM},
	{NO_FAN_DETECTED, NO_FAN_detected_FSM}
};
// state machine 
void FSM(void)
{
	if (State < Number_of_States)
	{
		(*StateMachine[State].func)();
	}
	else
	{
		#ifdef DEBUG
			Debug_UARTSend("Func FSM is greater than number of states\n");
		#endif
	}
}
