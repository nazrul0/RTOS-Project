#include "MKL25Z4.h"


#define RED_LED								18
#define GREEN_LED							19
#define BLUE_LED							1
#define MASK(x)								(1 << (x))


static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}


void InitGPIO(void) {
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK)| (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX Settings #include "Board_LED.h"  to make all 3 pins GPIO
	
	//! This seems to be clearing the last i bits
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	// Set the Data Direction Registers for PortB and PortD
	PTB->PDDR |= ( MASK( RED_LED ) | MASK( GREEN_LED ));
	PTD->PDDR |= MASK(BLUE_LED);
	
}
void TurnLEDOnAndOff(LED) {
	if(LED==RED_LED || LED==GREEN_LED) {
		PTB->PDOR &= ~MASK(LED);
	} else {
		PTD->PDOR &= ~MASK(LED);
	}
	
	delay(0xF0000);
	
	if(LED==RED_LED || LED==GREEN_LED) {
		PTB->PDOR |= MASK(LED);
	} else {
		PTD->PDOR |= MASK(LED);
	}
	
}

void controlLed(){
	int flag = 0;
	switch(flag) {
		case 0:
			TurnLEDOnAndOff(RED_LED);
			flag+=1;
		case 1:
			TurnLEDOnAndOff(BLUE_LED);
			flag+=1;
		case 2:
			TurnLEDOnAndOff(GREEN_LED);
			flag=0;
	}
}
unsigned int counter = 0;

// Main function


int main(void) {
	SystemCoreClockUpdate();
	InitGPIO();
}
