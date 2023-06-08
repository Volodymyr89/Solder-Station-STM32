#ifndef Short_Delays_us_ms
#define Short_Delays_us_ms

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f10x.h>
uint8_t Short_Delays_Systik_us_ms(uint32_t Delay_Value, uint8_t Delay_Range);
uint8_t Short_Delays_Systik_us_ms_Deglitch(uint32_t Delay_Value, uint8_t Delay_Range, uint8_t Input_to_Deglitch);
typedef enum {us, ms} Delay_Range;
#ifdef __cplusplus
}
#endif
 
#endif
