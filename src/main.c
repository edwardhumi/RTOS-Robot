/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

#define BAUD_RATE 9600
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
// Audio Pins
#define PTE29_Pin 29
#define MUSIC_MOD(x) (163840/x)
// UART Pins
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128


osThreadId_t redLED_Id, greenLED_id, blueLED_id;
osThreadId_t control_id, LED_running_id, LED_stop_id;
osThreadId_t motorForward_id, motorLeft_id, motorRight_id, motorBackward_id, motorStop_id, motorCurveLeft_id, motorCurveRight_id;
osThreadId_t audio1_id, audio2_id;
osThreadId_t control_id;

static volatile int audio_end;
static volatile int rightMethod = 0;
static volatile int leftMethod = 0;

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

/**** AUDIO ****/
const int NOTES_BDAY = 25;
const int TEMPO = 100;
const uint32_t TICKS_PER_MS = 20972; //20.9MHz

int happy_bday[] = {
	NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_F4, NOTE_E4,
	NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_F4, NOTE_E4,
	NOTE_C4, NOTE_C4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_E4, NOTE_D4,
	NOTE_AS4, NOTE_AS4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_F4
};

int canon[] = {
	NOTE_FS4,2, NOTE_E4,2,
	NOTE_D4,2, NOTE_CS4,2,
	NOTE_B3,2, NOTE_A3,2,
	NOTE_B3,2, NOTE_CS4,2,
	NOTE_FS4,2, NOTE_E4,2,
	NOTE_D4,2, NOTE_CS4,2,
	NOTE_B3,2, NOTE_A3,2,
	NOTE_B3,2, NOTE_CS4,2,
	NOTE_D4,2, NOTE_CS4,2,
	NOTE_B3,2, NOTE_A3,2,
	NOTE_G3,2, NOTE_FS3,2,
	NOTE_G3,2, NOTE_A3,2,

	NOTE_D4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8, 
	NOTE_A4,4, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, 
	NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,4, NOTE_FS3,8, NOTE_G3,8,
	NOTE_A3,8, NOTE_G3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,2,
	NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_G3,4, NOTE_FS3,8, NOTE_E3,8, 
	NOTE_FS3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,8, NOTE_B3,8,

	NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4, NOTE_CS4,8, NOTE_D4,8,
	NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2,
	NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4,
	NOTE_FS4,8, NOTE_G4,8, NOTE_A4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8,
	NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8,
	NOTE_FS4,8, NOTE_CS4,8, NOTE_A3,8, NOTE_A3,8,

	NOTE_CS4,4, NOTE_B3,4, NOTE_D4,8, NOTE_CS4,8, NOTE_B3,4,
	NOTE_A3,8, NOTE_G3,8, NOTE_A3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8,
	NOTE_A3,8, NOTE_B3,4, NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4,
	NOTE_CS4,8, NOTE_D4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8,
	NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2,  
};

const int CANON_COUNT = sizeof(canon) / sizeof(canon[0]);

void initAudio(void) {
	// USE PORT E FOR AUDIO
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->MOD = 7500;
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM0_C2SC &= ((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	audio_end = 0;
}

/**** UART ****/
static volatile uint8_t uart_data;

/* Init UART2 Interrupt */
void initUART2Interrupt(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_TIE_MASK) | (UART_C2_RIE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_TIE_MASK) | (UART_C2_RIE_MASK));
	uart_data = 0;
}

/* UART2 Interrupt Handler */
void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// do something to transmit data, UART2->D = ...
		UART2->C2 &= ~UART_C2_TIE_MASK; //in this case not transmitting data, so disable interrupt
	}
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		uart_data = UART2->D;
	}
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
  for (;;) {
		if (uart_data == 0x10) {
			// end
			osThreadFlagsSet(audio2_id, 0x1);
			osThreadFlagsSet(LED_stop_id, 0x1);
			if (!audio_end) {
				for (int i = 0; i < NOTES_BDAY; i++) {
					TPM0->MOD = MUSIC_MOD(happy_bday[i]);
					TPM0_C2V = MUSIC_MOD(happy_bday[i]) / 8; // Change denominator to adjust duty cycle for best sound
					delay(0x78640); // Adjust delay for beat
					TPM0_C2V = 0;
					delay(0x30580);
				}
			}
			audio_end = 1;
		}
		else {
			osThreadFlagsSet(audio1_id, 0x1);
		
			if (uart_data == 0x00) {
				osThreadFlagsSet(LED_stop_id, 0x1);
				osThreadFlagsSet(motorStop_id, 0x1);
			}
			else if (uart_data == 0x01) {
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorForward_id, 0x1);
			}
			else if (uart_data == 0x02) {
				leftMethod = 0;
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorLeft_id, 0x1);
			}
			else if (uart_data == 0x08) {
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorBackward_id, 0x1);
			}
			else if (uart_data == 0x04) {
				rightMethod = 0;
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorRight_id, 0x1);
			}
			else if (uart_data == 0x20) {
				leftMethod = 1;
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorLeft_id, 0x1);
			}
			else if (uart_data == 0x21) {
				rightMethod = 1;
				osThreadFlagsSet(LED_running_id, 0x1);
				osThreadFlagsSet(motorRight_id, 0x1);
			}
			else if (uart_data == 0x31) {
				//led_control(RED, 1);
				osThreadFlagsSet(redLED_Id, 0x1);
			}
			else if (uart_data == 0x30) {
				led_control(RED, 0);
			}
			else if (uart_data == 0x33) {
				led_control(GREEN, 1);
			}
			else if (uart_data == 0x32) {
				led_control(GREEN, 0);
			}
			else if (uart_data == 0x35) {
				led_control(BLUE, 1);
			}
			else if (uart_data == 0x34) {
				led_control(BLUE, 0);
			}
			else {
				//PTB->PDOR |= MASK(LED_RED);
				//PTB->PDOR |= MASK(LED_GREEN);
				//PTD->PDOR = MASK(LED_BLUE);
			}
		}
	}
}

void tRed (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		led_control(RED, 1);
		osDelay(1000);
		led_control(RED, 0);
		osDelay(1000);
	}
}

void tBlue (void *argument) {
	for(;;) {
		led_control(BLUE, 1);
		osDelay(1000);
		led_control(BLUE, 0);
		osDelay(1000);
	}
}

void tLED_running(void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		for (int i = FRONT_LED_START_PIN; i <= FRONT_LED_END_PIN; i++) {
			PTC->PDOR &= ~(1 << i);
    }
		for(int state = 1; state < 11; state++) {
			osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
			controlFrontLED(state);
			controlRearLED(1);
		}
	}
}

void tLED_stop(void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		controlFrontLED(0);
		controlRearLED(0);
	}
}

void tMotorForward (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		forward();
	}
}

void tMotorLeft (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		if (leftMethod == 1) {
			curveLeft();
		}
		else {
			left();
		}
	}
}

void tMotorBackward (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		backward();
	}
}

void tMotorRight (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		if (rightMethod == 1) {
			curveRight();
		}
		else {
			right();
		}
	}
}

void tMotorStop (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		stop();
	}
}

void tMotorCurveLeft (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		curveLeft();
	}
}

void tMotorCurveRight (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		curveRight();
	}
}

void tAudio1 (void *argument) {
	for(;;) {
		int wholenote = (60000 * 4) / TEMPO;
		uint32_t delayms;
		for (int i = 0; i < CANON_COUNT; i+=2) {
			osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
			if (!audio_end) {
				TPM0->MOD = MUSIC_MOD(canon[i]);
				TPM0_C2V = MUSIC_MOD(canon[i]) / 4; // Change denominator to adjust duty cycle for best sound
				delayms = wholenote / canon[i+1];
				delay(delayms * TICKS_PER_MS / 60); // magic divisor, sounds correct
				//osDelay(delayms / 2);
				TPM0_C2V = 0;
				TPM0->MOD = 0;
				delay(delayms * TICKS_PER_MS / 60);
				//osDelay(delayms / 2); 
			}
		}
	}
}

void tAudio2 (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		if (!audio_end) {
			for (int i = 0; i < NOTES_BDAY; i++) {
				TPM0->MOD = MUSIC_MOD(happy_bday[i]);
				TPM0_C2V = MUSIC_MOD(happy_bday[i]) / 4; // Change denominator to adjust duty cycle for best sound
				delay(0x78640); // Adjust delay for beat
				TPM0_C2V = 0;
				delay(0x30580);
			}
		}
		audio_end = 1;
	}
}

int main (void) {
  	// System Initialization
  	SystemCoreClockUpdate();
	initUART2Interrupt(BAUD_RATE);
	initGPIO();
	initLEDs();
	initPWM();
  	osKernelInitialize();                 // Initialize CMSIS-RTOS

	control_id = osThreadNew(tBrain, NULL, NULL);    // Create application main thread

	/** testing */
	redLED_Id = osThreadNew(tRed, NULL, NULL); // red wait for flag
	blueLED_id = osThreadNew(tBlue, NULL, NULL); // blue doesn't wait for flag, so it should continously run
	
	LED_running_id = osThreadNew(tLED_running, NULL, NULL);
	LED_stop_id = osThreadNew(tLED_stop, NULL, NULL);
	motorForward_id = osThreadNew(tMotorForward, NULL, NULL);
	motorLeft_id = osThreadNew(tMotorLeft, NULL, NULL);
	motorBackward_id = osThreadNew(tMotorBackward, NULL, NULL);
	motorRight_id = osThreadNew(tMotorRight, NULL, NULL);
	motorStop_id = osThreadNew(tMotorStop, NULL, NULL);
	audio1_id = osThreadNew(tAudio1, NULL, NULL);
//	audio2_id = osThreadNew(tAudio2, NULL, NULL);
//	motorCurveLeft_id = osThreadNew(tMotorCurveLeft, NULL, NULL);
//	motorCurveRight_id = osThreadNew(tMotorCurveRight, NULL, NULL);
  	osKernelStart();                      // Start thread execution
	
  	for (;;) {}
}
