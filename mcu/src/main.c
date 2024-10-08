// main.c
// Measure speed and direction of a motor
// Marina Ring
// mring@hmc.edu
// 10/6/2024

#include "main.h"

float motor_speed; 
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
    
    // Set pins as pull up, so when an encoder senses, it will output a digital low
    GPIOB->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD1, 0b01); // Set EncoderA as pull up
    // GPIOB->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD4, 0b01); // Set EncoderB as pull up
     GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD5, 0b01); // Set EncoderA as pull up

    // Initialize timers
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    initTIM(DELAY_TIM);
    initTIM(MEASURE_TIM);

    // Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTICR for encoder interrupts
    SYSCFG->EXTICR[0] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI1, 0b001); // set mux to select PB1
    //SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI4, 0b001); // set mux to select PB4
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR2_EXTI5, 0b000); // set mux to select PA5

    // Enable interrupts globally
    __enable_irq();

    // Configure interrupt for falling edge of encoder B input
    EXTI->IMR1 |= (1 << gpioPinOffset(ENCODER_B)); // Configure the mask bit
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(ENCODER_B));// Disable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(ENCODER_B));// Enable falling edge trigger
    //NVIC->ISER[0] |= (1 << EXTI4_IRQn); // turn on interrupt
    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn); // turn on interrupt

    //// Configure interrupt for falling edge of encoder A input
    EXTI->IMR1 |= (1 << gpioPinOffset(ENCODER_A)); // Configure the mask bit
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(ENCODER_A));// Disable rising edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(ENCODER_A));// Enable falling edge trigger
    NVIC->ISER[0] |= (1 << EXTI1_IRQn); // turn on interrupt

    // update reading every 200 milliseconds
    while(1) {
        // display result
        printf("Motor Speed: %f rev/s, Motor Direction: %s \n", motor_speed, (motor_direction) ? "CCW" : "CW");
        delay_millis(DELAY_TIM, 200);
    }

}

void EXTI1_IRQHandler(void){
    // Check that Encoder A triggered interrupt
    if (EXTI->PR1 & (1 << 1)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 1);

        // reset the counter
        MEASURE_TIM->EGR |= 1;     // Force update
        MEASURE_TIM->SR &= ~(0x1); // Clear UIF
    }
}

void EXTI9_5_IRQHandler(void){
    // Check that Encoder B triggered interrupt
    if (EXTI->PR1 & (1 << 5)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 5);

        uint32_t counter = MEASURE_TIM->CNT; // each count of the counter should be equivalent to 1 ms
        float pulse_length;
        float time_per_pulse;

        // check the counter and calculate speed and direction
        // if Encoder A is high at the same time as Encoder B, then the motor is spinning CCW
        if (~digitalRead(ENCODER_A)) {
            pulse_length = 4.0 * counter * 0.001;
            motor_direction = 1;
        } 
        // if Encoder A is low when Encoder B is high, then the motor is spinning CW
        else {
            pulse_length = 4.0/3.0 * counter * 0.001; 
            motor_direction = 0;
        }

        time_per_pulse = 1.0/pulse_length;
        motor_speed = time_per_pulse * 1.0/120.0; // there are 120 pulses per revolution

    }
}

//void EXTI4_IRQHandler(void){
//    // Check that Encoder B triggered interrupt
//    if (EXTI->PR1 & (1 << 4)){
//        // If so, clear the interrupt (NB: Write 1 to reset.)
//        EXTI->PR1 |= (1 << 4);

//        uint32_t counter = MEASURE_TIM->CNT; // each count of the counter should be equivalent to 1 ms
//        float pulse_length;
//        float time_per_pulse;

//        // check the counter and calculate speed and direction
//        // if Encoder A is high at the same time as Encoder B, then the motor is spinning CCW
//        if (~digitalRead(ENCODER_A)) {
//            pulse_length = 4 * counter * 0.001;
//            motor_direction = 1;
//        } 
//        // if Encoder A is low when Encoder B is high, then the motor is spinning CW
//        else {
//            pulse_length = 4/3 * counter * 0.001; 
//            motor_direction = 0;
//        }

//        time_per_pulse = 1/pulse_length;
//        motor_speed = time_per_pulse * 1/120; // there are 120 pulses per revolution

//    }
//}