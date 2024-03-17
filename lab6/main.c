#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "GPIO.h"

#define RED_LED 18      // PortB Pin 18
#define GREEN_LED 19    // PortB Pin 19
#define BLUE_LED 1      // PortD Pin 1
#define PTE20 20

typedef enum {LED_OFF, LED_ON} led_t;
typedef enum {RED, GREEN, BLUE} color_t;

/* Delay function */
static void delay(volatile uint32_t nof) {
  while (nof!=0) {
    __asm("NOP");
    nof--;
  }
}

GPIO_Type* getGPIO(PORT_Type* port) {
  if (port == PORTA)
    return PTA;
  else if (port == PORTB)
    return PTB;
  else if (port == PORTC)
    return PTC;
  else if (port == PORTD)
    return PTD;  
  else
    return PTE;
}

uint32_t getMaskPort(PORT_Type *port) {
  if (port == PORTA)
    return SIM_SCGC5_PORTA_MASK;
  else if (port == PORTB)
    return SIM_SCGC5_PORTB_MASK;
  else if (port == PORTC)
    return SIM_SCGC5_PORTC_MASK;
  else if (port == PORTD)
    return SIM_SCGC5_PORTD_MASK;
  else
    return SIM_SCGC5_PORTE_MASK;
}

void initGPIOPort(PORT_Type *port, uint8_t pin) {
  // Configure MUX settings to make all 3 pins GPIO
  // MUX_MASK = 011100000000, first step is to clear MUX
  // MUX(1) = 1 << 8, then set as GPIO
  // Configure Mode 1 for GPIO pin operation
  port->PCR[pin] &= ~PORT_PCR_MUX_MASK;
  port->PCR[pin] |= PORT_PCR_MUX(1);  
}

void InitGPIOAll(PORT_Type *port, uint8_t pins[], uint8_t numPins)
{
  // Used when setting DDR I/O
  uint32_t mask = 0;
  
  // Enable Clock to port
  // Controls the clock gate to the PORT modules
  SIM->SCGC5 |= getMaskPort(port);
  
  // Set all pins of port MUX to GPIO
  // Gets mask for all pins for DDR
  for (uint8_t i = 0; i < numPins; ++i) {
    initGPIOPort(port, pins[i]);
    mask |= MASK(pins[i]);
  }
  
  // Set Data Direction Registers for PortB and PortD
  // Configures pins as Input and Output
  getGPIO(port)->PDDR |= mask;
  
}

void setPin(PORT_Type *port, uint8_t pin, value_t value) {
  GPIO_Type *gpio = getGPIO(port);
  if (value == HIGH)
    gpio->PSOR |= MASK(pin);
  else
    gpio->PCOR |= MASK(pin);
}

void turnOffRGB() {
	/* Active low, so set bits to 1 to switch off */
	PTB->PSOR = MASK(RED_LED) | MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);
}

void led_control(color_t color) {
  turnOffRGB();
  /* Turn on specific LED colour, active low so set bits to 0 */
  switch(color) {
		case RED:
		 PTB->PCOR |= MASK(RED_LED);
		 break;
		case GREEN:
		 PTB->PCOR |= MASK(GREEN_LED);
		 break;
		case BLUE:
		 PTD->PCOR |= MASK(BLUE_LED);
		 break;
  }
}


void InitGPIO() {
  uint8_t pinsB[] = {RED_LED, GREEN_LED}, numPinsB = 2;
  uint8_t pinsD[] = {BLUE_LED}, numPinsD = 1;
  uint8_t pin[] = {PTE20};
  
  InitGPIOAll(PORTB, pinsB, numPinsB);
  InitGPIOAll(PORTD, pinsD, numPinsD);
  InitGPIOAll(PORTE, pin, 1);
}

 
/*----------------------------------------------------------------------------
 * Application led_red thread
 *---------------------------------------------------------------------------*/
void led_red_thread (void *argument) {
 
  // ...
  for (;;) {
    setPin(PORTE, PTE20, HIGH);
    led_control(RED);
    osDelay(1000);
    setPin(PORTE, PTE20, LOW);
    led_control(RED);
    osDelay(1000);    
  }
}

/*----------------------------------------------------------------------------
 * Application led_green thread
 *---------------------------------------------------------------------------*/
void led_green_thread (void *argument) {
 
  // ...
  for (;;) {
    
    led_control(GREEN);
    osDelay(1000);
    led_control(GREEN);
    osDelay(1000);    
  }
}
 
int main (void) {
  
  // System Initialization
  SystemCoreClockUpdate();
  // ...
  InitGPIO();
  turnOffRGB();
  setPin(PORTE, 20, LOW);
  
  
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(led_red_thread, NULL, NULL);    // Create application led_red thread
  osThreadNew(led_green_thread, NULL, NULL);    // Create application led_red thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}