// main.c
// Measure speed and direction of a motor
// Marina Ring
// mring@hmc.edu
// 10/6/2024

#include "main.h"

uint32_t motor_speed; 
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
    gpioEnable(GPIO_PORT_B);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);
    
    // Set pins as pull up, so when an encoder senses, it will output a digital low
    GPIOB->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD1, 0b01); // Set EncoderA as pull up
    GPIOB->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD7, 0b01); // Set EncoderB as pull up

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    initTIM(DELAY_TIM);
    initTIM(MEASURE_TIM);

    // Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTICR for encoder interrupts
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI2, 0b000);
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI3, 0b000);

    // Enable interrupts globally
    __enable_irq();

    // Configure interrupt for falling edge of encoder A input
    EXTI->IMR1 |= (1 << gpioPinOffset(ENCODER_A)); // Configure the mask bit
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(ENCODER_A));// Disable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(ENCODER_A));// Enable falling edge trigger
    NVIC->ISER[0] |= (1 << EXTI2_IRQn);

    // Configure interrupt for falling edge of encoder B input
    EXTI->IMR2 |= (1 << gpioPinOffset(ENCODER_B)); // Configure the mask bit
    EXTI->RTSR2 &= ~(1 << gpioPinOffset(ENCODER_B));// Disable rising edge trigger
    EXTI->FTSR2 |= (1 << gpioPinOffset(ENCODER_B));// Enable falling edge trigger
    NVIC->ISER[0] |= (1 << EXTI3_IRQn);

    // update reading every 200 milliseconds
    while(1) {
        // display result
        printf("Motor Speed: %f rev/s, Motor Direction: %s", motor_speed, (motor_direction) ? "CCW" : "CW");
        delay_millis(TIM2, 200);
    }

}

void EXTI2_IRQHandler(void){
    // Check that Encoder A triggered interrupt
    if (EXTI->PR1 & (1 << gpioPinOffset(ENCODER_A))){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << gpioPinOffset(ENCODER_A));

        // reset the counter
        TIM3->EGR |= 1;     // Force update
        TIM3->SR &= ~(0x1); // Clear UIF
        TIM3->CNT = 0;      // Reset count
    }
}

void EXTI3_IRQHandler(void){
    // Check that Encoder B triggered interrupt
    if (EXTI->PR1 & (1 << gpioPinOffset(ENCODER_B))){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << gpioPinOffset(ENCODER_B));

        uint32_t counter = TIM3->CNT; // each count of the counter should be equivalent to 1 ms
        uint32_t pulse_length;
        uint32_t time_per_pulse;

        // check the counter and calculate speed and direction
        // if Encoder A is high at the same time as Encoder B, then the motor is spinning CCW
        if (~digitalRead(ENCODER_A)) {
            pulse_length = 4 * counter;
            motor_direction = 1;
        } 
        // if Encoder A is low when Encoder B is high, then the motor is spinning CW
        else {
            pulse_length = 4/3 * counter; 
            motor_direction = 0;
        }

        time_per_pulse = 1/pulse_length;
        motor_speed = time_per_pulse * 1/120; // there are 120 pulses per revolution

    }
}