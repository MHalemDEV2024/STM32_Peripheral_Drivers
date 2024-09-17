/*
 * 002_led_button.c
 *
 *  Created on: Sep 17, 2024
 *  Author: Mohamed Saeed
 * 
 *  Description: This program toggles two LEDs on GPIOG pins 13 and 14 based on the state of a button connected to GPIOA pin 2. 
 *  When the button is pressed, the green LED (GPIOG_PIN_NO_13) toggles every second, and the red LED (GPIOG_PIN_NO_14) is turned off.
 *  When the button is not pressed, the red LED toggles every two seconds, and the green LED is turned off.
 */

#include "stm32f429xx.h"
#include "delay.h"

// Define constants for GPIO pins and delays
#define GREEN_LED_PIN    GPIO_PIN_NO_13
#define RED_LED_PIN      GPIO_PIN_NO_14
#define BUTTON_PIN       GPIO_PIN_NO_2
#define GREEN_LED_DELAY  1  // Delay in seconds for green LED
#define RED_LED_DELAY    2  // Delay in seconds for red LED

// Function prototypes
void System_GPIO_Init(void);
void toggle_leds(void);

int main(void) {
    // Initialize GPIO pins
    System_GPIO_Init();

    // Main loop: toggle LEDs based on button state
    while (1) {
        toggle_leds();
    }
}

/**
 * @brief Initializes GPIO pins for LEDs and button.
 */
void System_GPIO_Init(void) {
    // Enable clock for GPIOG and GPIOA
    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Configure GPIO pins for LEDs
    GPIO_PinInit(GPIOG, GREEN_LED_PIN, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    GPIO_PinInit(GPIOG, RED_LED_PIN, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);

    // Configure GPIO pin for button
    GPIO_PinInit(GPIOA, BUTTON_PIN, GPIO_MODE_IN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
}

/**
 * @brief Toggles LEDs based on the state of the button.
 */
void toggle_leds(void) {
    // Check button state
    if (GPIO_ReadFromInputPin(GPIOA, BUTTON_PIN)) {
        // Button pressed: toggle green LED
        GPIO_WriteToOutputPin(GPIOG, RED_LED_PIN, DISABLE);
        GPIO_ToggleOutputPin(GPIOG, GREEN_LED_PIN);  // Toggle green LED
        delay(GREEN_LED_DELAY);  // Delay for green LED
    } else {
        // Button not pressed: toggle red LED
        GPIO_WriteToOutputPin(GPIOG, GREEN_LED_PIN, DISABLE);
        GPIO_ToggleOutputPin(GPIOG, RED_LED_PIN);  // Toggle red LED
        delay(RED_LED_DELAY);  // Delay for red LED
    }
}
