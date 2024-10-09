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
// PWM Pins
#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
// LED Pins
#define FRONT_LED_START_PIN 0    // Start pin for front LED
#define FRONT_LED_END_PIN 9      // End pin for front LED
#define REAR_LED_PIN 10          // Pin for rear LED


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

/**** LED ****/
void initLEDs() {
	// Enable clock gating: turn on power for PORTC
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

	// Configure PORTC pins for LED output
	for (int i = FRONT_LED_START_PIN; i <= FRONT_LED_END_PIN; i++) {
		PORTC->PCR[i] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[i] |= PORT_PCR_MUX(1); // GPIO
		PTC->PDDR |= (1 << i); // Set pin as output
		PTC->PDOR &= ~(1 << i);
	}

	// Configure rear LED pin
	PORTC->PCR[REAR_LED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[REAR_LED_PIN] |= PORT_PCR_MUX(1); // GPIO
	PTC->PDDR |= (1 << REAR_LED_PIN); // Set pin as output
}

// Function to control front LEDs in running mode
void controlFrontLED(int state) {
	if(state == 0){
		for (int i = FRONT_LED_START_PIN; i <= FRONT_LED_END_PIN; i++) {
			PTC->PSOR |= (1 << i);
		}
	} else {
		int currentLED = FRONT_LED_START_PIN + state - 1;
		for (int i = FRONT_LED_START_PIN; i <= FRONT_LED_END_PIN; i++) {
			if (i!=currentLED) {
				PTC->PCOR |= (1 << i);
			}
			else {
				PTC->PSOR |= (1 << currentLED);
			}
		}
		delay(200000);
	}
}

// Function to control rear LED
void controlRearLED(int state) {
	if (state) {
		GPIOC->PSOR |= (1 << REAR_LED_PIN); // Turn on LED
		delay(233000);
		GPIOC->PCOR |= (1 << REAR_LED_PIN); // Turn off LED
		delay(233000);
	} else {
		GPIOC->PSOR |= (1 << REAR_LED_PIN); // Turn on LED
		delay(116500);
		GPIOC->PCOR |= (1 << REAR_LED_PIN); // Turn off LED
		delay(116500);
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

/**** PWM ****/
void initPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->MOD = 7500;
	TPM2->MOD = 7500;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM1_C0V = 0;
  	TPM1_C1V = 0; 
	TPM2_C0V = 0; 
  	TPM2_C1V = 0; 
}

void backward() {
	TPM1_C0V = 5000;
    TPM1_C1V = 0; 
	TPM2_C0V = 5000; 
    TPM2_C1V = 0; 
}

void forward() {
	TPM1_C0V = 0; 
    TPM1_C1V = 6000; 
	TPM2_C0V = 0; 
    TPM2_C1V = 6000; 
}

void right() {
	TPM1_C0V = 4500; 
    TPM1_C1V = 0; 
	TPM2_C0V = 0; 
    TPM2_C1V = 4500; 
}

void left() {
	TPM1_C0V = 0; 
    TPM1_C1V = 4500; 
	TPM2_C0V = 4500; 
    TPM2_C1V = 0; 
}

void stop() {
	TPM1_C0V = 0; 
    TPM1_C1V = 0; 
	TPM2_C0V = 0; 
    TPM2_C1V = 0;
}

void curveLeft() {
	TPM1_C0V = 0; 
  	TPM1_C1V = 7500; 
	TPM2_C0V = 0; 
  	TPM2_C1V = 1000; 
}

void curveRight() {
	TPM1_C0V = 0; 
    TPM1_C1V = 1000; 
	TPM2_C0V = 0; 
    TPM2_C1V = 7500; 
}

int main (void) {
  	// System Initialization
  	SystemCoreClockUpdate();
	initGPIO();
	initLEDs();
	initPWM();
  	osKernelInitialize();                 // Initialize CMSIS-RTOS
  	osKernelStart();                      // Start thread execution
	
  	for (;;) {}
}
