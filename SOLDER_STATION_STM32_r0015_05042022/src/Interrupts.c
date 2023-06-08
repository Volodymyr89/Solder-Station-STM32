#include "stm32f10x.h"                  // Device header
#include "Solder_Station_STM32.h" 
#include "interrupts.h"

volatile Updates_t Status_Updates_From_Interrupt_t = {.Show_SET_Solder_Data_to_Display_READY = STATUS_DISABLED, 
		                     .Show_SET_FAN_Data_to_Display_READY = STATUS_DISABLED, 
                         .RUN_Update_PID_Solder = STATUS_DISABLED, 
	                       .RUN_Update_PID_FAN = STATUS_DISABLED};

extern StateType State;
extern volatile status_t Skip_Button_Status;											 
FAN_Status FAN_Status_t = FAN_Deactivated;

// after timeout check 	state of FSM and deside to which state jump next									 
void TIM3_IRQHandler (void)
{			
	if ((TIM3->SR)&(TIM_SR_UIF))
	{
		TIM3->SR &= ~TIM_SR_UIF; /// always by zero 
		TIM3->CR1 &= ~TIM_CR1_CEN;
		
	while(1){
		
		if ((State == FAN || State == FAN_Speed) && (Monitor_Skip_Button (false) >=2) && (FAN_Status_t == FAN_Activated) || ((State == SETTING_TEMPERATURE_FAN) && (Monitor_Skip_Button (false) >=2) && (FAN_Status_t == FAN_Activated))){
			FAN_Status_t = CoolDown_and_OFF_FAN;
			  State = SOLDER; //State = STBY;
			break;
		}
	 else if ((State == SOLDER)&&(Monitor_Skip_Button (false) >=2)){
				FAN_Status_t = CoolDown_and_OFF_FAN; // enter coll down state
				State = STBY;
				break;
		}
		
	 else if (State == STBY){
				State = SOLDER; // enter SOLDER state
				break;
		}
	 else if ((State == SOLDER) || (State == SETTING_TEMPERATURE_SOLDER) || (State == NO_SOLDER_DETECTED)){
				State = FAN;
				break;
		}	
		else if ((State == FAN) || (State == SETTING_TEMPERATURE_FAN) || (State == NO_FAN_DETECTED)){
			State = FAN_Speed; // skip to FAN speed mode
			break;
		}
			else if ((State == SETTING_BARS_FAN))
		{
			State = FAN; // enter FAN state
			break;
		}
		else if ((State == FAN_Speed))
		{
			State = SOLDER;
			break;
		}
			else if ((State == SOLDER) || (State == SETTING_TEMPERATURE_SOLDER))
		{
			State = FAN;
			break;
		}				
}
		Monitor_Skip_Button(true);
		FSM();
}
}	                     

void TIM2_IRQHandler (void)
{
	volatile static uint8_t CNT =0;
	
	if ((TIM2->SR)&(TIM_SR_CC2IF))// check flag
	{
		TIM2->SR &= ~TIM_SR_CC2IF; // clear flag 
		CNT++;
		// set flag to run PID Solder
		if(CNT >= 2 && (Status_Updates_From_Interrupt_t.RUN_Update_PID_Solder == STATUS_DISABLED)){
			Status_Updates_From_Interrupt_t.RUN_Update_PID_Solder = STATUS_ENABLED;
			CNT = 0;	
		}
	}
}

void TIM1_CC_IRQHandler (void)
{
	if ((TIM1->SR)&(TIM_SR_CC3IF)) 
	{
		TIM1->SR &= ~TIM_SR_CC3IF;
		// set flag to run PID FAN
		if(Status_Updates_From_Interrupt_t.RUN_Update_PID_FAN == STATUS_DISABLED){
			Status_Updates_From_Interrupt_t.RUN_Update_PID_FAN = STATUS_ENABLED;
		}
	}
}
// ext interrupt for encoder to set temperatures for FAN and Solder
void EXTI9_5_IRQHandler (void)
{
	if ((EXTI->PR)&(EXTI_PR_PR7) || ((EXTI->PR)&(EXTI_PR_PR6))) // check STM state and pending register
 {
	if ((State == STBY || State == SETTING_TEMPERATURE_SOLDER || State == SOLDER) || (State == NO_SOLDER_DETECTED))
	{
		State = SETTING_TEMPERATURE_SOLDER;
		FSM();
	}	
	
	else if ((State == FAN || State == SETTING_TEMPERATURE_FAN) || (State == NO_FAN_DETECTED))
	{
		State = SETTING_TEMPERATURE_FAN;
		FSM();
	}
	
	else if (State == FAN_Speed || State== SETTING_BARS_FAN)
	{
		State = SETTING_BARS_FAN; // EXTI->PR |= EXTI_PR_PR6 | EXTI_PR_PR7;	
		FSM();
	}
}
}


void TIM4_IRQHandler (void)
{
			
	if ((TIM4->SR&TIM_SR_UIF))  // ALWAYS do if in interrupt!!!!!
	{
		if (State == SETTING_TEMPERATURE_FAN)
		{
			TIM4->SR &= ~TIM_SR_UIF; // clear interrupt flag
			TIM4->CR1 &= ~TIM_CR1_CEN;
			State=FAN;
			FSM();
		}
		
		else
		{
			TIM4->SR &= ~TIM_SR_UIF; // clear interrupt flag
			TIM4->CR1 &= ~TIM_CR1_CEN;
			State=SOLDER;
			FSM();
		}
	}		
}

