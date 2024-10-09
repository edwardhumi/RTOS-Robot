/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

#define MASK(x) (1UL << (x))
#define LED_RED (18)
#define LED_GREEN (19)
#define LED_BLUE (1)


/* Delay routine */
static void delay (volatile uint32_t nof) {
	while(nof != 0) {
		__asm("NOP");
		nof--;
	}
}

/*** LED CONTROL ***/
enum color_t {
	RED = 0,
	GREEN = 1,
	BLUE = 2
};

void led_control(enum color_t color, uint8_t on) {
	if (color == RED) {
		if (on)
			PTB->PDOR &= ~MASK(LED_RED);
		else
			PTB->PDOR |= MASK(LED_RED);
	}
	if (color == GREEN) {
		if (on)
			PTB->PDOR &= ~MASK(LED_GREEN);
		else
			PTB->PDOR |= MASK(LED_GREEN);
	}
	if (color == BLUE) {
		if (on)
			PTD->PDOR &= ~MASK(LED_BLUE);
		else
			PTD->PDOR |= MASK(LED_BLUE);
	}
}

void initGPIO() {
	//enable clock
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK);
	
	//set MUX to GPIO
	PORTB->PCR[LED_RED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LED_RED] |= PORT_PCR_MUX(1);
	PORTB->PCR[LED_GREEN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LED_GREEN] |= PORT_PCR_MUX(1);
	PORTD->PCR[LED_BLUE] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[LED_BLUE] |= PORT_PCR_MUX(1);
	
	//set DDR
	PTB->PDDR |= (MASK(LED_RED) | MASK(LED_GREEN));
	PTD->PDDR |= MASK(LED_BLUE);
	
	//turn off all LED
	led_control(RED, 0);
	led_control(BLUE, 0);
	led_control(GREEN, 0);
}

int main (void) {
  	// System Initialization
  	SystemCoreClockUpdate();
	initGPIO();
  	osKernelInitialize();                 // Initialize CMSIS-RTOS
  	osKernelStart();                      // Start thread execution
	
  	for (;;) {}
}
