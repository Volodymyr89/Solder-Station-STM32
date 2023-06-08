#include "Short_Delays_us_ms.h"

static __IO uint32_t usTicks_0; // For store tick counts in us
static __IO uint32_t usTicks_1; // For store tick counts in us



void SysTick_Handler(void) // SysTick_Handler function will be called every 1 us
{
    if (usTicks_0 != 0)
    {
        usTicks_0--;
    }
		 if (usTicks_1 != 0)
    {
        usTicks_1--;
    }
}	

uint8_t Short_Delays_Systik_us_ms(uint32_t Delay_Value, uint8_t Delay_Range)
{
	static uint8_t us_already_init = 0x00, ms_already_init = 0x00;
	uint8_t	ST=0;

	if (Delay_Range == us)
	{
		if (us_already_init == 0x00)
		{
		  SystemCoreClockUpdate();   // Update SystemCoreClock value
      SysTick_Config(SystemCoreClock / 1000000); // Configure the SysTick timer to overflow every 1 us
		  us_already_init = 0xFF;
			ms_already_init= 0x00;
		}
	}
	else if (Delay_Range == ms)
	{
		if (ms_already_init == 0x00)
		{
		  SystemCoreClockUpdate();   // Update SystemCoreClock value
      SysTick_Config(SystemCoreClock / 1000); // Configure the SysTick timer to overflow every 1 ms
		  ms_already_init=0xFF;
			us_already_init=0x00;
		}
	}

    // Reload us value
    usTicks_0 = Delay_Value;
    // Wait until usTick reach zero
    while (usTicks_0)
		{
			ST=0;
		}
		ST=1;
		return ST;
}

uint8_t Short_Delays_Systik_us_ms_Deglitch(uint32_t Delay_Value, uint8_t Delay_Range, uint8_t Input_to_Deglitch) 
{
	static uint8_t us_already_init = 0x00, ms_already_init = 0x00, Delay_Count_to_the_END = 0x00;

	if (Delay_Range == us)
	{
	
		if (us_already_init == 0x00)
		{
		  SystemCoreClockUpdate();   // Update SystemCoreClock value
      SysTick_Config(SystemCoreClock / 1000000); // Configure the SysTick timer to overflow every 1 us
		  us_already_init = 0xFF;
			ms_already_init= 0x00;
		}
	}
	else if (Delay_Range == ms)
	{
		if (ms_already_init == 0x00)
		{
		  SystemCoreClockUpdate();   // Update SystemCoreClock value
      SysTick_Config(SystemCoreClock / 1000); // Configure the SysTick timer to overflow every 1 ms
		  ms_already_init=0xFF;
			us_already_init=0x00;
		}
	}

   if (Input_to_Deglitch)
	{
		if (!Delay_Count_to_the_END)
	{
    usTicks_1 = Delay_Value;  // Reload us value
	
    // Wait until usTick reach zero
    while (usTicks_1)
		{
			  if (!Input_to_Deglitch)
				{
					usTicks_1 = Delay_Value;
				}
		}
		Delay_Count_to_the_END = 0xFF;
	}
		return 1;
	}
		else 
		{
			Delay_Count_to_the_END = 0x00;
			return 0;
		}
			
 }

 
		


