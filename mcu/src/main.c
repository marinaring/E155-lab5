// main.c
// Measure speed and direction of a motor
// Marina Ring
// mring@hmc.edu
// 10/6/2024

#include "main.h"

float motor_speed;
uint32_t counter1;
uint32_t counter2;
int motor_direction;

// Function used by printf to send characters to the laptop
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}


int main(void) {
    // Enable encoder as input
    gpioEnable(GPIO_PORT_A);
    gpioEnable(GPIO_PORT_B);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);
    

    // Initialize timers
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    initTIM(DELAY_TIM);
    initTIM_FAST(MEASURE_TIM1);
    initTIM_FAST(MEASURE_TIM2);


    // Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTICR for encoder interrupts
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI7, 0b001); // set mux to select PB7
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI9, 0b000); // set mux to select PA9


    // Enable interrupts globally
    __enable_irq();

    // Configure interrupt for falling edge of encoder B input
    EXTI->IMR1 |= (1 << gpioPinOffset(ENCODER_B)); // Configure the mask bit
    EXTI->RTSR1 |= (1 << gpioPinOffset(ENCODER_B));// Enable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(ENCODER_B));// Enable falling edge trigger

    //// Configure interrupt for falling edge of encoder A input
    EXTI->IMR1 |= (1 << gpioPinOffset(ENCODER_A)); // Configure the mask bit
    EXTI->RTSR1 |= (1 << gpioPinOffset(ENCODER_A));// Enable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(ENCODER_A));// Enable falling edge trigger

    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn); // turn on interrupt

    uint32_t prescaler, clock_freq;
    float pulse_length, time_per_pulse, average_speed;

    // update reading every 200 milliseconds
    while(1) {

        prescaler = MEASURE_TIM1 -> PSC;
        clock_freq = SystemCoreClock/prescaler;

        // check the counter and calculate speed and direction
        if (motor_direction) {
            pulse_length = 4.0/3.0 * ((float)(counter1 + counter2)/2.0);
        } 
        else {
            pulse_length = 4.0 * ((float)(counter1 + counter2)/2.0);
        }
        motor_speed = clock_freq/(120.0 * pulse_length); // there are 120 pulses per revolution
        
        // display result
        printf("Motor Speed: %f rev/s, Motor Direction: %s \n", motor_speed, (motor_direction) ? "CCW" : "CW");
        delay_millis(DELAY_TIM, 200);
    }

}



void EXTI9_5_IRQHandler(void){
    // Check that Encoder A triggered interrupt
    if (EXTI->PR1 & (1 << 7)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 7);
        
        // if rising edge triggered
        if (digitalRead(ENCODER_A)) {
          // reset the counter
          MEASURE_TIM2->EGR |= 1;     // Force update
          MEASURE_TIM2->SR &= ~(0x1); // Clear UIF
        // if falling edge triggered
        } else {
          // reset the counter
          MEASURE_TIM1->EGR |= 1;     // Force update
          MEASURE_TIM1->SR &= ~(0x1); // Clear UIF
        }
    }

    // Check that Encoder B triggered interrupt
    if (EXTI->PR1 & (1 << 9)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 9);
        
        // if rising edge triggered
        if (digitalRead(ENCODER_B)) {
          counter2 = MEASURE_TIM2->CNT; // each count of the counter should be equivalent to 1 ms
          
          // for rising edge triggered scenario
          // if Encoder A is low when Encoder B is high, then the motor is spinning CCW
          if (~digitalRead(ENCODER_A)) {
              motor_direction = 1;
          } else { 
              motor_direction = 0;
          }
        } 
        // if falling edge triggered
        else {
          counter1 = MEASURE_TIM1->CNT; // each count of the counter should be equivalent to 1 ms
        
          // for falling edge triggered scenario
          // if Encoder A is high when Encoder B is high, then the motor is spinning CCW
          if (digitalRead(ENCODER_A)) {
              motor_direction = 1;
          } else { 
              motor_direction = 0;
          }
        }
    }
}
