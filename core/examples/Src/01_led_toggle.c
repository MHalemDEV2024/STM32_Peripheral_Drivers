/*
 * 001_led_toggle.c
 *
 *  Created on: Sep 16, 2024 
 *      Author: Mohamed Saeed
 *
 *  Description:
 *  This program toggles two LEDs connected to GPIO pins 13 and 14 of port G
 *  on an STM32F429xx microcontroller. The LEDs toggle alternately with a configurable delay
 *  between each toggle.
 */

#include "stm32f429xx.h"
#include "delay.h"



int main(void) {

    // Enable clock for GPIOG
    GPIO_PeriClockControl(GPIOG, ENABLE);


    // Initialize individual GPIO pins
    GPIO_PinInit(GPIOG, GPIO_PIN_NO_13, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    GPIO_PinInit(GPIOG, GPIO_PIN_NO_14, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);

    // Main loop: toggle LEDs alternately with a configurable delay
    while (1) {

        GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);  // Toggle green LED
        delay(1);  // Delay for 1 second
        GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_14);  // Toggle red LED
        delay(2);  // Delay for 2 seconds

    }
}
