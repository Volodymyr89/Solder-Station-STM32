#include "stm32f10x.h"     
#include "SPI_Transmitt.h"
#include "STM32_GPIO_lib.h"
#include "Solder_Station_STM32.h"

extern StateType State;
extern solder_connect_status_t solder_connect_status;
extern fan_connect_status_t fan_connect_status;
// init SPI
void SPI2_init(void){
	 RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	 SPI2->CR1 = 0x0000;
	 SPI2->CR2 = 0x0000;
	 SPI2->CR1 &= ~SPI_CR1_DFF;
	 SPI2->CR1 &= ~SPI_CR1_CPOL;
	 SPI2->CR1 &= ~SPI_CR1_CPHA;
	 SPI2->CR1 &= ~SPI_CR1_CRCEN;
	 SPI2->CR1 &= ~SPI_CR1_LSBFIRST;
	 SPI2->CR1 |= SPI_CR1_SSM;
	 SPI2->CR1 |= SPI_CR1_SSI;
	 SPI2->CR1 &= ~SPI_CR1_BR;
	 SPI2->CR1 |= SPI_CR1_MSTR;
	 SPI2->CR2 |= SPI_CR2_TXDMAEN;
	 SPI2->CR1 |= SPI_CR1_SPE;

	GPIO_EN (GPIO_PINB,  PIN13_CRH, AF_Alternate_Function_Push_Pull_50MHz); // Clock
	GPIO_EN (GPIO_PINB,  PIN15_CRH, AF_Alternate_Function_Push_Pull_50MHz); // MOSI
	GPIO_EN (GPIO_PINB,  PIN10_CRH, Output_Push_Pull_2MHz); // DC
	GPIO_EN (GPIO_PINB,  PIN1_CRL, Output_Push_Pull_2MHz);	// RESET
	 
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		DMA1_Channel5->CCR = 0x00; 
		DMA1_Channel5->CCR |= DMA_CCR5_PL; 
		DMA1_Channel5->CCR &= ~DMA_CCR5_MSIZE;
		DMA1_Channel5->CCR |= DMA_CCR5_PSIZE_0; // 16 bit
		DMA1_Channel5->CCR &= ~DMA_CCR5_PINC;
		DMA1_Channel5->CCR |= DMA_CCR5_DIR;
		DMA1_Channel5->CPAR = (uint32_t)(&SPI2->DR); 			
}
	
// send command to SSD1306 mode
void SPI2_SEND_Command (uint8_t DATA_to_send)
{ 
	   DMA1_Channel5->CMAR = (uint32_t)&DATA_to_send;
		 DMA1_Channel5->CNDTR = 1;
		 DMA1_Channel5->CCR |= DMA_CCR5_EN; 
	  while((SPI2->SR & SPI_SR_TXE) && (!(SPI2->SR & SPI_SR_BSY)))
	 	{}
		DMA1_Channel5->CCR &= ~DMA_CCR5_EN;
}


// receive temperature from MAX31855 via SPI
float SPI1_Receive_Temperature(Chose_Data_From_t Solder_or_Fan_data){
	
	static bool SPI1_init= false;
	 uint16_t SPI1_Received_Temperature_Data=0, SPI1_Received_Temperature_Data_data_taken=0;
	 float returnValue = 0.0;

if (false == SPI1_init){
	 SPI1_init = true;
	 RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	 SPI1->CR1 = 0x0000;
	 SPI1->CR2 = 0x0000;
	 SPI1->CR1 |= SPI_CR1_SSM;
   SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // MSB first
	 SPI1->CR1 |= SPI_CR1_SSI;
	 SPI1->CR1 |= SPI_CR1_DFF;
	 SPI1->CR1 |= SPI_CR1_CPHA; // must read on falling edge according to datasheet
	 SPI1->CR1 |= SPI_CR1_BR;
	 SPI1->CR1 |= SPI_CR1_MSTR;		
  GPIO_EN (GPIO_PINA,  PIN5_CRL, AF_Alternate_Function_Push_Pull_50MHz); // clock
	GPIO_EN (GPIO_PINA,  PIN6_CRL, AF_Alternate_Function_Push_Pull_50MHz); //MISO
	GPIO_EN (GPIO_PINA,  PIN3_CRL, Output_Push_Pull_2MHz); // CS for Solder
	GPIO_EN (GPIO_PINA,  PIN4_CRL, Output_Push_Pull_2MHz);	// CS for FAN
	GPIOA-> ODR |= GPIO_ODR_ODR3; // init to HIGH by default
	GPIOA-> ODR |= GPIO_ODR_ODR4; // init to HIGH by default
	
 SPI1->CR1 |= SPI_CR1_SPE;
}
	volatile	uint8_t count =50;
	if(Solder_or_Fan_data==SOLDER_Chose){
	  GPIOA-> ODR &= ~GPIO_ODR_ODR4;//SELECT_SOLDER_TEMPERATURE_SENSOR;
		}
	else if (Solder_or_Fan_data==FAN_Chose){
		GPIOA-> ODR &= ~GPIO_ODR_ODR3;//SELECT_FAN_TEMPERATURE_SENSOR;
	}
			while(count>0){count--;};
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_SR_RXNE)){}
		count=50;
	while(count>0){count--;};
	GPIOA-> ODR |= GPIO_ODR_ODR4;//DESELECT_SOLDER_TEMPERATURE_SENSOR;
	GPIOA-> ODR |= GPIO_ODR_ODR3;//DESELECT_FAN_TEMPERATURE_SENSOR;
	SPI1_Received_Temperature_Data = (uint16_t)SPI1->DR;
	SPI1_Received_Temperature_Data_data_taken =SPI1_Received_Temperature_Data&(uint16_t)0x7FFC;
	SPI1_Received_Temperature_Data_data_taken>>=2;
	returnValue = ((float)(SPI1_Received_Temperature_Data_data_taken)*(float)0.25);
	return (returnValue);
}
	







