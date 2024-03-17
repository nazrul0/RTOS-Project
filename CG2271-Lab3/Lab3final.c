#include "MKL25Z4.h"

unsigned int counter = 0;
unsigned int on = 0;
unsigned int led_num = 0;

#define RED_LED			18 	// PortB Pin 18
#define GREEN_LED		19  // PortB Pin 19
#define BLUE_LED 		1   // PortD Pin 1
#define SW_POS			6	 	// PortD Pin 6
#define MASK(x)			(1 << (x))


/* Delay Function */

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

void initSwitch(void) 
{
	// enable clock for PortD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	/* Select GPIO and enable pull-up resistors and interrupts on 
	falling edges of pin connected to switch*/
	PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1) |
												PORT_PCR_PS_MASK |
												PORT_PCR_PE_MASK |
												PORT_PCR_IRQC(0x0a));
	
	// Set PORT D Switch Bit to input
	PTD->PDDR &= ~MASK(SW_POS);
	
	// Enable Interrupts

	NVIC_EnableIRQ(PORTD_IRQn);
}

void initLED(void) 
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX settings to make all 3 pins GPIO
	
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);

	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);

	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);

	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

void Led_on() 
{
	switch(led_num) {
			case 0:
				PTD->PDOR |= MASK(BLUE_LED);
				PTB->PDOR &= ~MASK(RED_LED);
				led_num = 1;
				break;
			case 1:
				PTB->PDOR |= MASK(RED_LED);
				PTB->PDOR &= ~MASK(GREEN_LED);
				led_num = 2;
				break;
			case 2:
				PTB->PDOR |= MASK(GREEN_LED);
				PTD->PDOR &= ~MASK(BLUE_LED);
				led_num = 0;
				break;
	}
}

void PORTD_IRQHandler()
{
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	counter++;
	Led_on();	
	
	// Clear INT Flag
	PORTD->ISFR |= MASK(SW_POS);
}

/* MAIN function */

int main(void)
{
	initSwitch();
	initLED();
	while(1)
	{}
}