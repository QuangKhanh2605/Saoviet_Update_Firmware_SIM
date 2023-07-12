#include "user_bootloader.h"

void Bootloader_Jump_Firmware(uint32_t address)
{
	/* Turn off peripheral , Clear interrupt flag*/
	HAL_RCC_DeInit();  
	
	 /* C pending interrupt request, Turn off system tick */
	HAL_DeInit();
	
	/* Turn off fault harder */
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
	
	/* Set Main Stack Poiter */
	__set_MSP(*((volatile uint32_t*) address));
	
	uint32_t jumpaddress = *((volatile uint32_t*) (address + 4));
	
	/* Set Program Counter to Application Address */
	void (*reset_handler)(void) = (void(*)(void))(jumpaddress);
	reset_handler();
}

