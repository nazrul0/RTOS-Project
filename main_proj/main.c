/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "MKL25Z4.h"
/* Default Core Clk Freq is 20.97152MHz */
// Current code will run at 48 MHz core clk freq and 24MHz Bus clk freq

#define NOTE_SILENT 0
#define NOTE_c 261
#define NOTE_d 294
#define NOTE_e 329
#define NOTE_f 349
#define NOTE_g 391
#define NOTE_gS 415
#define NOTE_a 440
#define NOTE_aS 455
#define NOTE_b 466
#define NOTE_cH 523
#define NOTE_cSH 554
#define NOTE_dH 587
#define NOTE_dSH 622
#define NOTE_eH 659
#define NOTE_fH 698
#define NOTE_fSH 740
#define NOTE_gH 784
#define NOTE_gSH 830
#define NOTE_aH 880

#define RUNNING_SONG_NOTE_NUM 20
#define RUNNING_SONG_NOTES_LEN 70

#define SUCCESS_SONG_NOTES_LEN 10

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23 // Page 162 datasheet
#define UART2_INT_PRIO 128

#define FORWARD 0x01  
#define REVERSE 0x00

#define STOP 0x000
#define SPEED_01 0x01
#define SPEED_02 0x02
#define SPEED_03 0x03
#define SPEED_07 0x04
#define SPEED_08 0x05
#define SPEED_09 0x06
#define SPEED_10 0x07

#define PTB0_Pin 0			// right reverse
#define PTB1_Pin 1			// right forward
#define PTB2_Pin 2			// left forward
#define PTB3_Pin 3			// left reverse

#define PTC1_Pin 1
#define PTC2_Pin 2

#define PTD5_Pin 5
#define PTD0_Pin 0
#define PTD2_Pin 2
#define PTD3_Pin 3
#define PTD6_Pin 6
#define PTE30_Pin 30
#define PTE29_Pin 29
#define PTE22_Pin 22
#define PTE21_Pin 21
#define PTE20_Pin 20
#define PTE1_Pin 1

#define MASK(x) (1 << (x))

#define DUTYCYCLE_00 0x000
#define DUTYCYCLE_10 0x2EE			// 7500*0.1
#define DUTYCYCLE_20 0x5DC			// 7500*0.2
#define DUTYCYCLE_30 0x8CA			// 7500*0.3
#define DUTYCYCLE_40 0xBB8			// 7500*0.4
#define DUTYCYCLE_50 0xEA6			// 7500*0.5
#define DUTYCYCLE_60 0x1194			// 7500*0.6
#define DUTYCYCLE_70 0x1482			// 7500*0.7
#define DUTYCYCLE_80 0x1770			// 7500*0.8
#define DUTYCYCLE_90 0x1A5E			// 7500*0.9
#define DUTYCYCLE_100 0x1D4C			// 7500*1.0

#define MOVEMENT 0x0
#define SUCCESS 0x1
#define MOVING 0x01
#define STATIONARY 0x00
#define TRUE 0x1
#define FALSE 0x0

#define ESP_SUCCESS_MSG 0x67

uint8_t green_leds[] = {PTD6_Pin, PTD3_Pin, PTD2_Pin, PTD0_Pin, PTD5_Pin, 
PTE22_Pin, PTE30_Pin, PTE29_Pin, PTE21_Pin, PTE20_Pin};
	
typedef struct {
	uint8_t cmd;
	uint8_t data;
} myDataPkt;

//! State
volatile int STATE = 0;
volatile int SUCCESS_STATE = 0;

//osMutexId_t myMutex;
osSemaphoreId_t mySem;
osMessageQueueId_t movementMsg;
osMessageQueueId_t successMsg;

unsigned int rightDirection, leftDirection = FORWARD;
//unsigned int rightDirection = FORWARD;
//unsigned int leftDirection = FORWARD;


unsigned int rightSpeed, leftSpeed = STOP;
//unsigned int rightSpeed = SPEED_08;
//unsigned int leftSpeed = SPEED_02;

unsigned int successState = 0;

unsigned int tpm1c0v, tpm1c1v, tpm2c0v, tpm2c1v = DUTYCYCLE_00; 


volatile unsigned int rxData = 0x00;

unsigned int runningSongNotes[] = {NOTE_a, NOTE_a, NOTE_a, NOTE_f, NOTE_cH, NOTE_a, NOTE_f, NOTE_cH, NOTE_a, NOTE_eH, NOTE_eH, NOTE_eH, NOTE_fH, NOTE_cH, NOTE_gS, NOTE_f, NOTE_cH, NOTE_a, NOTE_aH, NOTE_a, NOTE_a, NOTE_aH, NOTE_gSH, NOTE_gH, NOTE_fSH, NOTE_fH, NOTE_fSH, NOTE_SILENT, NOTE_aS, NOTE_dSH, NOTE_dH, NOTE_cSH, NOTE_cH, NOTE_b, NOTE_cH, NOTE_SILENT, NOTE_f, NOTE_gS, NOTE_f, NOTE_a, NOTE_cH, NOTE_a, NOTE_cH, NOTE_eH, NOTE_aH, NOTE_a, NOTE_a, NOTE_aH, NOTE_gSH, NOTE_gH, NOTE_fSH, NOTE_fH, NOTE_fSH, NOTE_SILENT, NOTE_aS, NOTE_dSH, NOTE_dH, NOTE_cSH, NOTE_cH, NOTE_b, NOTE_cH, NOTE_SILENT, NOTE_f, NOTE_gS, NOTE_f, NOTE_cH, NOTE_a, NOTE_f, NOTE_c, NOTE_a};

unsigned int runningSongDurations[] = {500, 500, 500, 350, 150, 500, 350, 150, 1000, 500, 500, 500, 350, 150, 500, 350, 150, 1000, 500, 350, 150, 500, 250, 250, 125, 125, 250, 250, 250, 500, 250, 250, 125, 125, 250, 250, 125, 500, 375, 125, 500, 375, 125, 1000, 500, 350, 150, 500, 250, 250, 125, 125, 250, 250, 250, 500, 250, 250, 125, 125, 250, 250, 250, 500, 375, 125, 500, 375, 125, 1000};

unsigned int successSongNotes[] = {NOTE_c, NOTE_aH, NOTE_c, NOTE_aH, NOTE_c, NOTE_aH, NOTE_c, NOTE_aH, NOTE_c, NOTE_aH};

unsigned int successSongDurations[] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150};

unsigned int success_set = FALSE;

// Init UART2
void initUART2(uint32_t baud_rate) {
  uint32_t divisor, bus_clock;
  
  // Enable clocking to the two different peripheral block
  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  
  // Page 162 datasheet (Alt 4 GPIO config) to receive
  PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
  
  // TE and RE is to enable both transmitter and receiver
  // Clearing because we are setting up the module (setting to 0)
  UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
  
  // Default System Clock is 48MHz [system clock clocks to CPU]
  // Bus clock is to clock some subsystem, just how some system is designed
  // UART runs at half the system clock rate
  bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
  divisor = bus_clock / (baud_rate * 16); // [* 16 because oversampling internally to account for some noise -> unique to this controller]
  UART2->BDH = UART_BDH_SBR(divisor >> 8); // BDH = Buad Rate High
  UART2->BDL = UART_BDL_SBR(divisor); // BDL = Baud Rate low
  
  UART2->C1 = 0; // No parity so set to 0
  UART2->S2 = 0; // Not configuring anything
  UART2->C3 = 0; // In case you want to send a 9th bit data then can use this. Can enable interrupts if you want to handle errors
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
  
  // Turn on and receive and receive interrupt
  UART2->C2 |= ((UART_C2_RIE_MASK) | (UART_C2_RE_MASK));
}  

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rxData = UART2->D;
	}
//	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
//		UART2->S1 &=  ~(UART_S1_OR_MASK | UART_S1_PF_MASK | UART_S1_FE_MASK);
//	}
}

void initPWM(void) {
	// Enable Clock to PORTB
	// Need to enable port power because need to configure the registers for the MUX
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Enable Clock to PORTC
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Configure MUX settings to make pins GPIO
	// Configure Mode 3 for the new PWM pin operation
	// Pin 0
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 3 (chip specific) Timer Module 1 Channel 0
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	// Pin 1
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 3 (chip specific) Timer Module 1 Channel 1
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	// Pin 2
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 3 (chip specific) Timer Module 2 Channel 0
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	// Pin 3
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 3 (chip specific) Timer Module 2 Channel 1
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	// Configure Mode 4 for the PORTC PWM pin operation
	// Pin C1
	PORTC->PCR[PTC1_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 4 (chip specific) Timer Module 0 Channel 0
	PORTC->PCR[PTC1_Pin] |= PORT_PCR_MUX(4);
	
	// Pin C2
	PORTC->PCR[PTC2_Pin] &= ~PORT_PCR_MUX_MASK;
	// Alternative 4 (chip specific) Timer Module 0 Channel 1
	PORTC->PCR[PTC2_Pin] |= PORT_PCR_MUX(4);
	
	// Enable Clock for Timer 1
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	// Enable Clock for Timer 2
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	// Enable Clock for Timer 0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	// Select Clock for TPM module
	// TPMSRC = Clk source [using MCGFLLCLK]
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// Set Modulo value to 48000000 (48 Mhz) / 128 (Prescaler) = 375000 
	// 375000 / 7500 = 50 Hz [basically when reach 7500 (max count) will flip over to 0 and start again]
	TPM1->MOD = 7500;
	TPM2->MOD = 7500;
	TPM0->MOD = 7500;

	// Edge-Aligned PWM
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	// Set CMOD to 1 == LPTPM counter increments on every LPTPM counter clock
	// PS 7 is 128 prescaler
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	// Set CMOD to 1 == LPTPM counter increments on every LPTPM counter clock
	// PS 7 is 128 prescaler
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	// Set CMOD to 1 == LPTPM counter increments on every LPTPM counter clock
	// PS 7 is 128 prescaler
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM1 Channel 0 -> PTB0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM1 Channel 1 -> PTB1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM2 Channel 0 -> PTB2
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM2 Channel 1 -> PTB3
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 0 -> PTC1
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 1 -> PTC2
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initGPIO(void){
	// Enable Clock to PORTD. PORTE has already been enabled in initUART2
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);
	
	// Configure MUX settings to make all LED pins GPIO
	// Clear pins and set PCR[led] to 1 for GPIO mode
	for (int i = 0; i < 5; i++) {
		PORTD->PCR[green_leds[i]] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[green_leds[i]] |= PORT_PCR_MUX(1);
	}
	
	for (int i = 5; i < 10; i++) {
		PORTE->PCR[green_leds[i]] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[green_leds[i]] |= PORT_PCR_MUX(1);
	}
	
	PORTE->PCR[PTE1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE1_Pin] |= PORT_PCR_MUX(1);

	
	// Set Data Direction Registers for PortB and PortD
	PTD->PDDR |= (MASK(PTD5_Pin) | MASK(PTD0_Pin) | MASK(PTD2_Pin) | MASK(PTD3_Pin) | MASK(PTD6_Pin));
	PTE->PDDR |= (MASK(PTE30_Pin) | MASK(PTE29_Pin) | MASK(PTE22_Pin) | MASK(PTE21_Pin) | MASK(PTE20_Pin) | MASK(PTE1_Pin));
}



/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

void tBrain (void *argument) {
 
  // ...
  for (;;) {
//    osMutexAcquire(myMutex, osWaitForever);
		osSemaphoreAcquire(mySem, osWaitForever);
    
		// process rx_data into wheel direction and duty cycles

		leftDirection = (rxData >> 7) & 0x01;
		leftSpeed = (rxData >> 4) & 0x07; // Adjusting mask to extract 3 bits
		rightDirection = (rxData >> 3) & 0x01;
		rightSpeed = rxData & 0x07; // Adjusting mask to extract 3 bits
		
		successState = rxData & 0xFF;
		
		myDataPkt txMovementDataPkt;
		myDataPkt txSuccessDataPkt;
		
//		txSuccessDataPkt.cmd = SUCCESS;
//		txSuccessDataPkt.data = FALSE;
//		SUCCESS_STATE = 0;
		
		if (successState == ESP_SUCCESS_MSG) {
			txSuccessDataPkt.cmd = SUCCESS;
			txSuccessDataPkt.data = TRUE;
			SUCCESS_STATE = 1;
		} else {
			txSuccessDataPkt.cmd = SUCCESS;
			txSuccessDataPkt.data = FALSE;
			SUCCESS_STATE = 0;
		}
		
		if ((leftSpeed | rightSpeed) == STOP) {
			txMovementDataPkt.cmd = MOVEMENT;
			txMovementDataPkt.data = STATIONARY;
			STATE = 0;
		} else {
			txMovementDataPkt.cmd = MOVEMENT;
			txMovementDataPkt.data = MOVING;
			STATE = 1;
		}
		
		osMessageQueuePut(movementMsg, &txMovementDataPkt, NULL, 0);
		osMessageQueuePut(successMsg, &txSuccessDataPkt, NULL, 0);
    
//    osMutexRelease(myMutex);
		osSemaphoreRelease(mySem);
  }
}

void tMotorControl (void *argument) {
 
  // ...
  for (;;) {
//    osMutexAcquire(myMutex, osWaitForever);
		osSemaphoreAcquire(mySem, osWaitForever);
		
		switch (rightSpeed) {
			case STOP:
				tpm1c0v = DUTYCYCLE_00;
				tpm1c1v = DUTYCYCLE_00;
				break;
			case SPEED_01:
				tpm1c0v = DUTYCYCLE_10;
				tpm1c1v = DUTYCYCLE_10;
				break;
			case SPEED_02:
				tpm1c0v = DUTYCYCLE_20;
				tpm1c1v = DUTYCYCLE_20;
				break;
			case SPEED_03:
				tpm1c0v = DUTYCYCLE_30;
				tpm1c1v = DUTYCYCLE_30;
				break;
//			case SPEED_04:
//				tpm1c0v = DUTYCYCLE_40;
//				tpm1c1v = DUTYCYCLE_40;
//				break;
//			case SPEED_05:
//				tpm1c0v = DUTYCYCLE_50;
//				tpm1c1v = DUTYCYCLE_50;
//				break;
//			case SPEED_06:
//				tpm1c0v = DUTYCYCLE_60;
//				tpm1c1v = DUTYCYCLE_60;
//				break;
			case SPEED_07:
				tpm1c0v = DUTYCYCLE_70;
				tpm1c1v = DUTYCYCLE_70;
				break;
			case SPEED_08:
				tpm1c0v = DUTYCYCLE_80;
				tpm1c1v = DUTYCYCLE_80;
				break;
			case SPEED_09:
				tpm1c0v = DUTYCYCLE_90;
				tpm1c1v = DUTYCYCLE_90;
				break;
			case SPEED_10:
				tpm1c0v = DUTYCYCLE_100;
				tpm1c1v = DUTYCYCLE_100;
				break;
		}
		
		switch (leftSpeed) {
			case STOP:
				tpm2c0v = DUTYCYCLE_00;
				tpm2c1v = DUTYCYCLE_00;
				break;
			case SPEED_01:
				tpm2c0v = DUTYCYCLE_10;
				tpm2c1v = DUTYCYCLE_10;
				break;
			case SPEED_02:
				tpm2c0v = DUTYCYCLE_20;
				tpm2c1v = DUTYCYCLE_20;
				break;
			case SPEED_03:
				tpm2c0v = DUTYCYCLE_30;
				tpm2c1v = DUTYCYCLE_30;
				break;
//			case SPEED_04:
//				tpm2c0v = DUTYCYCLE_40;
//				tpm2c1v = DUTYCYCLE_40;
//				break;
//			case SPEED_05:
//				tpm2c0v = DUTYCYCLE_50;
//				tpm2c1v = DUTYCYCLE_50;
//				break;
//			case SPEED_06:
//				tpm2c0v = DUTYCYCLE_60;
//				tpm2c1v = DUTYCYCLE_60;
//				break;
			case SPEED_07:
				tpm2c0v = DUTYCYCLE_70;
				tpm2c1v = DUTYCYCLE_70;
				break;
			case SPEED_08:
				tpm2c0v = DUTYCYCLE_80;
				tpm2c1v = DUTYCYCLE_80;
				break;
			case SPEED_09:
				tpm2c0v = DUTYCYCLE_90;
				tpm2c1v = DUTYCYCLE_90;
				break;
			case SPEED_10:
				tpm2c0v = DUTYCYCLE_100;
				tpm2c1v = DUTYCYCLE_100;
				break;
		}
		
		switch (rightDirection) {
			case REVERSE:
				TPM1_C0V = tpm1c0v;
				TPM1_C1V = DUTYCYCLE_00;
				break;
			case FORWARD:
				TPM1_C0V = DUTYCYCLE_00;
				TPM1_C1V = tpm1c1v;
		}
		
		switch (leftDirection) {
			case REVERSE:
				TPM2_C0V = tpm2c0v;
				TPM2_C1V = DUTYCYCLE_00;
				break;
			case FORWARD:
				TPM2_C0V = DUTYCYCLE_00;
				TPM2_C1V = tpm2c1v;
		}
    
//    osMutexRelease(myMutex);
		osSemaphoreRelease(mySem);
  }
}

void tRedLed (void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);

		myDataPkt rxMovementDataPkt;
		osMessageQueueGet(movementMsg, &rxMovementDataPkt, NULL, osWaitForever);
		
		if (rxMovementDataPkt.data == MOVING) {
			// 500ms
			PTE->PSOR |= MASK(PTE1_Pin); 
			osDelay(500);
			PTE->PCOR |= MASK(PTE1_Pin);
			osDelay(500);
		} 
		else if (rxMovementDataPkt.data == STATIONARY) {
			// 250ms
			PTE->PSOR |= MASK(PTE1_Pin);
			osDelay(250);
			PTE->PCOR |= MASK(PTE1_Pin);
			osDelay(250);
		}
		
		osSemaphoreRelease(mySem);
  }
}

void tGreenLed (void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);

		myDataPkt rxMovementDataPkt;
		osMessageQueueGet(movementMsg, &rxMovementDataPkt, NULL, osWaitForever);
		
		
		
		if (rxMovementDataPkt.data == MOVING) {
			// one at a time
			//! Turn off first
			PTD->PCOR |= (MASK(PTD5_Pin) | MASK(PTD0_Pin) | MASK(PTD2_Pin) | MASK(PTD3_Pin) | MASK(PTD6_Pin));
			PTE->PCOR |= (MASK(PTE30_Pin) | MASK(PTE29_Pin) | MASK(PTE22_Pin) | MASK(PTE21_Pin) | MASK(PTE20_Pin));
			
			for (int i = 0; i < 5; i++) {
				if(STATE == 0) {
					break;
				}
				
				PTD->PSOR |= MASK(green_leds[i]);
				osDelay(100);
				PTD->PCOR |= MASK(green_leds[i]);
				osDelay(100);
			}
			
			for (int i = 5; i < 10; i++) {
				if(STATE == 0) {
					break;
				}
				PTE->PSOR |= MASK(green_leds[i]);
				osDelay(100);
				PTE->PCOR |= MASK(green_leds[i]);
				osDelay(100);
			}
		} 
		else if (rxMovementDataPkt.data == STATIONARY) {
			// all at once
			PTD->PSOR |= (MASK(PTD5_Pin) | MASK(PTD0_Pin) | MASK(PTD2_Pin) | MASK(PTD3_Pin) | MASK(PTD6_Pin));
			PTE->PSOR |= (MASK(PTE30_Pin) | MASK(PTE29_Pin) | MASK(PTE22_Pin) | MASK(PTE21_Pin) | MASK(PTE20_Pin));
		}
		
		osSemaphoreRelease(mySem);
  }
}

void tBuzzer (void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySem, osWaitForever);

		myDataPkt rxSuccessDataPkt;
		osMessageQueueGet(successMsg, &rxSuccessDataPkt, NULL, osWaitForever);
	
		TPM0_C0V = 3750;
		
		if (rxSuccessDataPkt.data == TRUE && success_set == FALSE) {			
			uint32_t tpm0modVal = 7500;
			
			for (int i = 0; i < SUCCESS_SONG_NOTES_LEN; i++) {
				if (SUCCESS_STATE == 0) break;
				
				tpm0modVal = 375000 / successSongNotes[i];
				TPM0->MOD = tpm0modVal;
				
				TPM0_C0V = tpm0modVal/2;
				osDelay(successSongDurations[i]);
//				TPM0_C0V = 0;
//				osDelay(successSongDurations[i]);
			}
			
			success_set = TRUE;			
			
		} 
		else if (rxSuccessDataPkt.data == FALSE) {
			uint32_t tpm0modVal = 7500;
			
			for (int i = 0; i < RUNNING_SONG_NOTES_LEN; i++) {
				if (SUCCESS_STATE == 1) break;
				
				tpm0modVal = 375000 / runningSongNotes[i];
				TPM0->MOD = tpm0modVal;
				
				TPM0_C0V = tpm0modVal/2;
				osDelay(runningSongDurations[i]);
				TPM0_C0V = 0;
				osDelay(runningSongDurations[i]);
			}
		}
		
		osSemaphoreRelease(mySem);
  }
}

int main(void) {
	// System Initialization
  SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initPWM();
	initGPIO();
	
	// Channels 1 to 3 are for fR, fL, rR, rL wheels, all 
	TPM1_C0V = DUTYCYCLE_80;
	TPM1_C1V = DUTYCYCLE_80;
	TPM2_C0V = DUTYCYCLE_80;
	TPM2_C1V = DUTYCYCLE_80;
 
  osKernelInitialize();                				// Initialize CMSIS-RTOS
//	myMutex = osMutexNew(NULL);
	mySem = osSemaphoreNew(5,5,NULL);
	movementMsg = osMessageQueueNew(1, sizeof(myDataPkt), NULL);
	successMsg = osMessageQueueNew(1, sizeof(myDataPkt), NULL);
	osThreadNew(tBrain, NULL, NULL);    				// Create tBrain thread
	osThreadNew(tMotorControl, NULL, NULL);    	// Create tMotorControl thread
	osThreadNew(tRedLed, NULL, NULL);    				// Create tRedLed thread
	osThreadNew(tGreenLed, NULL, NULL);    			// Create tGreenLed thread
	osThreadNew(tBuzzer, NULL, NULL);        		// Create tBuzzer thread
  osKernelStart();                      			// Start thread execution
  for (;;) {}
}

