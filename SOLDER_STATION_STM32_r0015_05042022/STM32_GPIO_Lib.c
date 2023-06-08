#include "STM32_GPIO_lib.h"
 
enum GPIO_SIDE GPIOx;
enum GPIO_Set GPIO_Setting; // declaring
enum PIN_Number_ PIN_Number_CRL_CRH;

void GPIO_EN ( uint8_t GPIOx, uint8_t PIN_Number_CRL_CRH, uint8_t GPIO_Setting)
{
	uint8_t GPIO_Setting_Conv, PIN_Number_Conv,PIN_Number_Conv_for_pullup_down;
	
	if (GPIO_Setting == AF_Alternate_Function_Push_Pull_2MHz || GPIO_Setting == AF_Alternate_Function_Push_Pull_10MHz || GPIO_Setting == AF_Alternate_Function_Push_Pull_50MHz || GPIO_Setting == AF_Alternate_Function_Open_Drain_2MHz || GPIO_Setting == AF_Alternate_Function_Open_Drain_10MHz || GPIO_Setting == AF_Alternate_Function_Open_Drain_50MHz)
		{
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
		}
	
switch (GPIOx)
{
  case GPIO_PINA:
  
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

		if (PIN_Number_CRL_CRH <= 0x07)
	{	
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOA->ODR &= ~1<< (PIN_Number_CRL_CRH);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOA->ODR |= 1<< (PIN_Number_CRL_CRH);
		}
		
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		
		GPIOA->CRL &=~(0x0F << (PIN_Number_CRL_CRH * 4)); // clear 4 bits before setting
		GPIOA->CRL |= GPIO_Setting_Conv << PIN_Number_CRL_CRH * 4; // configure PIN
	}
		
	else 
	{
		PIN_Number_Conv = PIN_Number_CRL_CRH & 0x0F;
		PIN_Number_Conv_for_pullup_down = PIN_Number_CRL_CRH;
		PIN_Number_Conv_for_pullup_down &= ~(1<<4);
		PIN_Number_Conv_for_pullup_down |= 1<<3;
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOA->ODR &= ~1<< (PIN_Number_Conv_for_pullup_down);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOA->ODR |= 1<< (PIN_Number_Conv_for_pullup_down);
		}
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOA->CRH &=~(0x0F << (PIN_Number_Conv * 4)); // clear 4 bits before setting
		GPIOA->CRH |= GPIO_Setting_Conv << PIN_Number_Conv * 4; // configure PIN
 }
  break;
	
 case GPIO_PINB:
  
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		if (PIN_Number_CRL_CRH <= 0x07)
	{	
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOB->ODR &= ~1<< (PIN_Number_CRL_CRH);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOB->ODR |= 1<< (PIN_Number_CRL_CRH);
		}
		
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOB->CRL &=~(0x0F << (PIN_Number_CRL_CRH * 4)); // clear 4 bits before setting
		GPIOB->CRL |= GPIO_Setting_Conv << PIN_Number_CRL_CRH * 4; // configure PIN
	}
		
	else 
	{
		PIN_Number_Conv = PIN_Number_CRL_CRH & 0x0F;
		PIN_Number_Conv_for_pullup_down = PIN_Number_CRL_CRH;
		PIN_Number_Conv_for_pullup_down &= ~(1<<4);
		PIN_Number_Conv_for_pullup_down |= 1<<3;
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOB->ODR &= ~1<< (PIN_Number_Conv_for_pullup_down);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOB->ODR |= 1<< (PIN_Number_Conv_for_pullup_down);
		}
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOB->CRH &=~(0x0F << (PIN_Number_Conv * 4)); // clear 4 bits before setting
		GPIOB->CRH |= GPIO_Setting_Conv << PIN_Number_Conv * 4; // configure PIN
 }
  break;
	
 case GPIO_PINC:
  
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

		if (PIN_Number_CRL_CRH <= 0x07)
	{	
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOC->ODR &= ~1<< (PIN_Number_CRL_CRH);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOC->ODR |= 1<< (PIN_Number_CRL_CRH);
		}
		
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOC->CRL &=~(0x0F << (PIN_Number_CRL_CRH * 4)); // clear 4 bits before setting
		GPIOC->CRL |= GPIO_Setting_Conv << PIN_Number_CRL_CRH * 4; // configure PIN
	}
		
else 
	{
		PIN_Number_Conv = PIN_Number_CRL_CRH & 0x0F;
		PIN_Number_Conv_for_pullup_down = PIN_Number_CRL_CRH;
		PIN_Number_Conv_for_pullup_down &= ~(1<<4);
		PIN_Number_Conv_for_pullup_down |= 1<<3;
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOC->ODR &= ~1<< (PIN_Number_Conv_for_pullup_down);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOC->ODR |= 1<< (PIN_Number_Conv_for_pullup_down);
		}
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOC->CRH &=~(0x0F << (PIN_Number_Conv * 4)); // clear 4 bits before setting
		GPIOC->CRH |= GPIO_Setting_Conv << PIN_Number_Conv * 4; // configure PIN
 }
  break;
 
 case GPIO_PIND:
  
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;

		if (PIN_Number_CRL_CRH <= 0x07)
	{	
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOD->ODR &= ~1<< (PIN_Number_CRL_CRH);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOD->ODR |= 1<< (PIN_Number_CRL_CRH);
		}
		
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOD->CRL &=~(0x0F << (PIN_Number_CRL_CRH * 4)); // clear 4 bits before setting
		GPIOD->CRL |= GPIO_Setting_Conv << PIN_Number_CRL_CRH * 4; // configure PIN
	}
		
	else 
	{
		PIN_Number_Conv = PIN_Number_CRL_CRH & 0x0F;
		PIN_Number_Conv_for_pullup_down = PIN_Number_CRL_CRH;
		PIN_Number_Conv_for_pullup_down &= ~(1<<4);
		PIN_Number_Conv_for_pullup_down |= 1<<3;
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOD->ODR &= ~1<< (PIN_Number_Conv_for_pullup_down);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOD->ODR |= 1<< (PIN_Number_Conv_for_pullup_down);
		}
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOD->CRH &=~(0x0F << (PIN_Number_Conv * 4)); // clear 4 bits before setting
		GPIOD->CRH |= GPIO_Setting_Conv << PIN_Number_Conv * 4; // configure PIN
 }
  break;
	
 case GPIO_PINE:
  
		RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

		if (PIN_Number_CRL_CRH <= 0x07)
	{	
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOE->ODR &= ~1<< (PIN_Number_CRL_CRH);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOE->ODR |= 1<< (PIN_Number_CRL_CRH);
		}
		
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOE->CRL &=~(0x0F << (PIN_Number_CRL_CRH * 4)); // clear 4 bits before setting
		GPIOE->CRL |= GPIO_Setting_Conv << PIN_Number_CRL_CRH * 4; // configure PIN
	}
		
else 
	{
		PIN_Number_Conv = PIN_Number_CRL_CRH & 0x0F;
		PIN_Number_Conv_for_pullup_down = PIN_Number_CRL_CRH;
		PIN_Number_Conv_for_pullup_down &= ~(1<<4);
		PIN_Number_Conv_for_pullup_down |= 1<<3;
		if (GPIO_Setting == Input_Pull_Down)
		{
			GPIOE->ODR &= ~1<< (PIN_Number_Conv_for_pullup_down);
		}
	else if (GPIO_Setting == Input_Pull_Up)
		{
			GPIOE->ODR |= 1<< (PIN_Number_Conv_for_pullup_down);
		}
		GPIO_Setting_Conv = GPIO_Setting & 0x0F;
		GPIOE->CRH &=~(0x0F << (PIN_Number_Conv * 4)); // clear 4 bits before setting
		GPIOE->CRH |= GPIO_Setting_Conv << PIN_Number_Conv * 4; // configure PIN
 }
  break;
}	
	
}

void 	GPIO_DIS (GPIO_TypeDef* GPIOx, char PIN_Number)
{

  if (GPIOx == GPIOA)
  {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPAEN;
	}
	
	if (GPIOx == GPIOB)
  {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPBEN;
	}
	
	 if (GPIOx == GPIOC)
  {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPCEN;
	}
	
	 if (GPIOx == GPIOD)
  {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPDEN;
	}
	 if (GPIOx == GPIOE)
  {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPEEN;
	}
	

	if (PIN_Number <= 0x07){
		
	GPIOx->CRL &= ~(0x0F << (PIN_Number * 4));
	GPIOx->CRL |= 0x04 << (PIN_Number*4);
	GPIOx->ODR &= ~1<< PIN_Number;
	}		
	
 if ((PIN_Number >0x07) && (PIN_Number<=0x17)){
		
	uint8_t Shift;
	 
	Shift = PIN_Number & 0x0F;
	GPIOx->CRH &= ~(0x0F << (Shift*4));
	GPIOx->CRH |= 0x04 << (Shift*4);
	GPIOx->ODR &= ~1<< (PIN_Number - 0x02);
}
	}
