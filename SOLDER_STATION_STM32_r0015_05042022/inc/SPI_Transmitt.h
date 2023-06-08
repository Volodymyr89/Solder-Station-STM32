#ifndef _SPI__Tansmitt_H
#define _SPI__Tansmitt_H

#include "stm32f10x.h"   
#include <stdbool.h>

extern bool SPI2_init_Status;
extern bool SPI1_init_Status;
typedef enum{
	SOLDER_Chose,
	FAN_Chose
}Chose_Data_From_t;

volatile typedef enum{
  solder_is_disconected,
	solder_is_conected,
	soldercheckdone
}solder_connect_status_t;

typedef enum{
  fan_is_disconected,
	fan_is_conected,
	fancheckdone
}fan_connect_status_t;

// function prototypes
void SPI2_init(void);
void SPI1_init(void);
float SPI1_Receive_Temperature(Chose_Data_From_t Solder_or_Fan_data);
void SPI2_SEND_Command (uint8_t DATA_to_send);
void DMA1_Channel5_init(void);
extern bool Check;
extern uint8_t Count;

#endif
