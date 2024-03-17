/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

#define RED_LED                18
#define GREEN_LED              19
#define BLUE_LED              1
#define MASK(x)                (1 << (x))

osMutexId_t myMutex;

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

void turnLEDOn(int LED) {
  if(LED==RED_LED | LED==GREEN_LED) {
    PTB->PDOR &= ~MASK(LED);
  } else {
    PTD->PDOR &= ~MASK(LED);
  }
}
  

void turnLEDOff(int LED) {
  if(LED==RED_LED | LED==GREEN_LED) {
    PTB->PDOR |= MASK(LED);
  } else {
    PTD->PDOR |= MASK(LED);
  }
}

void offRGB(void) {
  PTB->PDOR |= MASK(RED_LED);
  PTB->PDOR |= MASK(GREEN_LED);
  PTD->PDOR |= MASK(BLUE_LED);
}

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
//__NO_RETURN static void app_main (void *argument) {
//  (void)argument;
//  // ...
//  for (;;) {
//    turnLEDOn(RED_LED);
//    osDelay(1000);
//    turnLEDOff(RED_LED);
//    osDelay(1000);
//  }
//}

///*----------------------------------------------------------------------------
// * Application led_red thread
// *---------------------------------------------------------------------------*/
__NO_RETURN static void led_red_thread (void *argument) {
//  (void)argument;
//  // ...
  for (;;) {
		//osMutexAcquire(myMutex, osWaitForever);

		turnLEDOn(RED_LED);
		osDelay(1000);
		//delay(0x80000);
		turnLEDOff(RED_LED);
		//osDelay(1000);
		delay(0x80000);

		//osMutexRelease(myMutex);
  }
}

///*----------------------------------------------------------------------------
// * Application led_green thread
// *---------------------------------------------------------------------------*/
__NO_RETURN static void led_green_thread (void *argument) {
  //(void)argument;
  // ...
  for (;;) {
    //osMutexAcquire(myMutex, osWaitForever);
    
    turnLEDOn(GREEN_LED);
    osDelay(1000);
    //delay(0x80000);
    turnLEDOff(GREEN_LED);
    //osDelay(1000);
    delay(0x80000);
    
    //osMutexRelease(myMutex);
  }
}

const osThreadAttr_t thread_attr = {
  .priority = osPriorityAboveNormal1
};
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  InitGPIO();
  offRGB();    // double check this
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  //myMutex = osMutexNew(NULL);
  osThreadNew(led_red_thread, NULL, &thread_attr);    // Create application main thread
  osThreadNew(led_green_thread, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}