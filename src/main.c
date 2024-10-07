// main.c
// Measure speed and direction of a motor
// Marina Ring
// mring@hmc.edu
// 10/6/2024

#include "main.h"

int main(void) {
    // TODO: figure out display
    // For right now, will view output in the terminal

    // Enable encoder as input
    gpioEnable(GPIO_PORT_B);
    pinMode(ENCODER_A, GPIO_INPUT);
    pinMode(ENCODER_B, GPIO_INPUT);
    
    // TODO: set pins as pull up
    // GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b01); // Set PA2 as pull-up

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    initTIM(DELAY_TIM);

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
    EXTI->IMR2 |= (1 << gpioPinOffset(ENCODER_A)); // Configure the mask bit
    EXTI->RTSR2 &= ~(1 << gpioPinOffset(ENCODER_A));// Disable rising edge trigger
    EXTI->FTSR2 |= (1 << gpioPinOffset(ENCODER_A));// Enable falling edge trigger
    NVIC->ISER[0] |= (1 << EXTI3_IRQn);

    while(1){   
        delay_millis(TIM2, 200);
    }

}

void EXTI2_IRQHandler(void){
    // Check that the button was what triggered our interrupt
    if (EXTI->PR1 & (1 << )){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << );

        // Then toggle the LED
        togglePin(LED_PIN);

    }
}

void EXTI3_IRQHandler(void){
    // Check 
    
}