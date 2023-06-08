#include "Solder_Station_STM32.h" 
#include "SPI_Transmitt.h"
#include "STM32_GPIO_lib.h"
#include "SSD1306.h"
#include "interrupts.h"

// chose board or debugger with UART
#define DEBUG
//#define BOARD

// functions prorotypes
void PB0_LED_main_STATUS(void);
void Status_LED_init(void);
// end of functions prorotypes 

// global variables
extern volatile bool Wait_and_Turn_OFF_FAN;
volatile bool Wait_and_Turn_OFF_FAN = false;
// end of global variables

//enumerations and structures
extern  Updates_t Status_Updates_From_Interrupt_t;
volatile StateType State;
solder_connect_status_t solder_connect_status = {solder_is_conected};
fan_connect_status_t  fan_connect_status = {fan_is_conected};
// end of enumerations and structures


int main (){
	#ifdef DEBUG
		Debug_UART_init();
	#endif
	
  Status_LED_init();  // init LED	
  SPI2_init(); // init SPI2
  ENCODER_init(); // init rotary encoder based on HDD inductance motor
  Skip_Button_init(); // skip button interrupt init
  ssd1306_RESET(); // keep before SSD1306_Init()
  SSD1306_Init(); // init OLED
  Make_Click_init(); // init sound indication for rotary encoder
  State= STBY;   // chose first state after start up
  FSM();        //start FSM
	
  while(1){
	 //check flag from interrupt and show data from solder to OLED
		if ((Status_Updates_From_Interrupt_t.Show_SET_Solder_Data_to_Display_READY == STATUS_ENABLED) && ((SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY))))
	{
		 Make_Click();
		 Show_SET_Solder_Data_to_Display();
		 Status_Updates_From_Interrupt_t.Show_SET_Solder_Data_to_Display_READY = STATUS_DISABLED;
	}
	 //check flag from interrupt and show data from FAN to OLED
	else if ((Status_Updates_From_Interrupt_t.Show_SET_FAN_Data_to_Display_READY == STATUS_ENABLED)  && (SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY)))
	{
    Make_Click();		
    Show_SET_FAN_Data_to_Display();	
		Status_Updates_From_Interrupt_t.Show_SET_FAN_Data_to_Display_READY = STATUS_DISABLED;
  }
	 //check flag from interrupt and run PID for solder
	 if (Status_Updates_From_Interrupt_t.RUN_Update_PID_Solder == STATUS_ENABLED)//  && (SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY)))
	 {
		  Status_Updates_From_Interrupt_t.RUN_Update_PID_Solder = STATUS_DISABLED;
		  Update_PID_Solder();
		 if (State == SOLDER){
		  Data_Averager_For_Solder();
		 }
		}
	 //check flag from interrupt and run PID for FAN
	else if ((Status_Updates_From_Interrupt_t.RUN_Update_PID_FAN == STATUS_ENABLED)  && (SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY)))
	 {
		  Status_Updates_From_Interrupt_t.RUN_Update_PID_FAN = STATUS_DISABLED;
		  Update_PID_FAN();
		 if (State == FAN){
		  Data_Averager_From_FAN();
		 }
	 }
	  //check flag from interrupt and show FAN speed bars on OLED
	 	else if ((Status_Updates_From_Interrupt_t.RUN_Update_Duty_Cycle_Bars == STATUS_ENABLED)  && (SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY)))
			{
			  Status_Updates_From_Interrupt_t.RUN_Update_Duty_Cycle_Bars = STATUS_DISABLED;
			  Make_Click();
			  Show_SET_FAN_Bars_Data_to_Display();
		}
		// monitor button pressed
		if (!((TIM3->SR)&(TIM_SR_UIF)))
		{
			Monitor_Skip_Button(false);
		}
		// monitor if FAN was turned OFF and wait it will be cooled down
			Turn_OFF_FAN ();
			PB0_LED_main_STATUS();
	}
}	

// indicator PC13 LED
void Status_LED_init(void){
	#ifdef BOARD
		GPIO_EN (GPIO_PINC,  PIN13_CRH, Output_Push_Pull_10MHz);
	#else
		GPIO_EN (GPIO_PINB,  PIN0_CRL, Output_Push_Pull_10MHz);
	#endif
		
}
// LED to check if while loop is running, if yes then blink LED
void PB0_LED_main_STATUS(void)
{
	static uint32_t Count =0;
	Count++;
	
	if (Count>50000&&Count<60000)
	{
		#ifdef BOARD
			GPIOC->ODR &= ~GPIO_ODR_ODR13;
		#else
			GPIOB->ODR |= GPIO_ODR_ODR0;
		#endif
	}
	
	else if(Count>60000)
	{
		Count = 0;
		#ifdef BOARD
			GPIOC->ODR |= GPIO_ODR_ODR13;
		#else
			GPIOB->ODR &= ~GPIO_ODR_ODR0;
		#endif
	}
}
