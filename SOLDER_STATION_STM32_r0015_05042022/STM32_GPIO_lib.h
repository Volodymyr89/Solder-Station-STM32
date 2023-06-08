#ifndef _GPIO_lib_H
#define _GPIO_lib_H

#include <stm32f10x.h>


enum PIN_Number_
{
	PIN0_CRL = 0x00,
	PIN1_CRL = 0x01, 
	PIN2_CRL = 0x02,
	PIN3_CRL = 0x03,
	PIN4_CRL = 0x04, 
	PIN5_CRL = 0x05, 
	PIN6_CRL = 0x06, 
	PIN7_CRL = 0x07, 
	
	PIN8_CRH =  0x10,
	PIN9_CRH =  0x11, 
	PIN10_CRH = 0x12,
	PIN11_CRH = 0x13,
	PIN12_CRH = 0x14,
	PIN13_CRH = 0x15,
	PIN14_CRH = 0x16, 
	PIN15_CRH = 0x17, 
};


 enum GPIO_Set
{
	Input_Analog			 	         = 0x00,
	Input_Floating		      		 =	0x04,
	Input_Pull_Down               = 0x018,// to recognise when pullup down
	Input_Pull_Up                 = 0x028, // to recognise when pullup down
	Output_Push_Pull_2MHz         = 0x02,
	Output_Push_Pull_10MHz        = 0x01,
	Output_Push_Pull_50MHz        = 0x03,
	Output_Open_Drain_2MHz        = 0x06,
	Output_Open_Drain_10MHz       = 0x05,
	Output_Open_Drain_50MHz       = 0x07,
	AF_Alternate_Function_Push_Pull_2MHz  = 0x0A,
	AF_Alternate_Function_Push_Pull_10MHz  = 0x09,
	AF_Alternate_Function_Push_Pull_50MHz  = 0x0B,
	AF_Alternate_Function_Open_Drain_2MHz = 0x0E,
	AF_Alternate_Function_Open_Drain_10MHz = 0x0D,
	AF_Alternate_Function_Open_Drain_50MHz = 0x0F,
};


enum GPIO_SIDE {GPIO_PINA = 0x00, GPIO_PINB = 0x01, GPIO_PINC = 0x02, GPIO_PIND = 0x03, GPIO_PINE = 0x04};


void GPIO_EN ( uint8_t GPIOx, uint8_t PIN_Number_CRL_CRH, uint8_t GPIO_Setting);
//void GPIO_DIS (GPIO_TypeDef* GPIOx, char PIN_Number);
 
#endif
