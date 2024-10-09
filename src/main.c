/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

int main (void) {
  	// System Initialization
  	SystemCoreClockUpdate();
  	osKernelInitialize();                 // Initialize CMSIS-RTOS
  	osKernelStart();                      // Start thread execution
	
  	for (;;) {}
}
