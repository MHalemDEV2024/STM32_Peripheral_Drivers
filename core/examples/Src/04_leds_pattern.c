/*
 * 004_led_button.c
 *
 *  Created on: Sep 17, 2024
 *  Author: ADMIN
 * 
 *  Description: This program controls a series of LEDs connected to GPIOG pins 1 through 8. 
 *  It shifts the LEDs left or right based on the state of a button connected to GPIOA pin 2. 
 *  LEDs shift in blocks with a configurable delay time and shift size.
 */

#include "stm32f429xx.h"
#include "delay.h"

#define LEFT_SHIFT   0
#define RIGHT_SHIFT  1

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON  = 1
} LEDState;

// Function prototypes
void System_GPIO_Init(void);
void ShiftLEDs(uint8_t startPin, uint8_t shiftCount, uint32_t delayTime, uint8_t direction, uint8_t shiftSize);
void SetLEDState(uint8_t startPin, uint8_t endPin, LEDState state);

int main(void) {
    // Initialize GPIO pins
    System_GPIO_Init();

    // Main loop: shift LEDs based on button state
    while (1) {
        if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_2)) {
            // Shift LEDs to the left in blocks of 2 with a 2-second delay
            ShiftLEDs(1, 8, 2, LEFT_SHIFT, 2); // Delay in 2 seconds 
        } else {
            // Shift LEDs to the right in blocks of 1 with a 1-second delay
            ShiftLEDs(8, 8, 1, RIGHT_SHIFT, 1); // Delay in 1 seconds 
        }
    }
}

/**
 * @brief Initializes GPIO pins for LEDs and button.
 */
void System_GPIO_Init(void) {
    // Enable clock for GPIOG and GPIOA
    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Initialize GPIO pins for LEDs (GPIOG, Pins 1 to 8)
    for (uint8_t i = 1; i <= 8; i++) {
        GPIO_PinInit(GPIOG, i, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    }

    // Initialize GPIO pin for button (GPIOA, Pin 2)
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_2, GPIO_MODE_IN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
}

/**
 * @brief Shifts LEDs in a specified direction with a delay.
 * 
 * @param startPin   The starting pin number for shifting.
 * @param shiftCount The total number of pins to shift.
 * @param delayTime  The delay time in milliseconds between shifts.
 * @param direction  The direction of shifting (LEFT_SHIFT or RIGHT_SHIFT).
 * @param shiftSize  The size of the shift block.
 */
void ShiftLEDs(uint8_t startPin, uint8_t shiftCount, uint32_t delayTime, uint8_t direction, uint8_t shiftSize) {
    // Validate parameters
    if (startPin < 1 || startPin > 8 || shiftSize < 1 || shiftSize > shiftCount) {
        return; // Invalid parameters; exit function
    }

    // Perform shifting
    if (direction == LEFT_SHIFT) {
        for (uint8_t j = 0; j < shiftCount; j += shiftSize) {
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_STATE_ON);
            delay(delayTime);
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_STATE_OFF);
        }
    } else if (direction == RIGHT_SHIFT) {
        for (int8_t j = 0; j < shiftCount; j += shiftSize) {
            SetLEDState(startPin - j - shiftSize + 1, startPin - j, LED_STATE_ON);
            delay(delayTime);
            SetLEDState(startPin - j - shiftSize + 1, startPin - j, LED_STATE_OFF);
        }
    }
}

/**
 * @brief Sets the state of LEDs in a specified range.
 * 
 * @param startPin The starting pin number.
 * @param endPin   The ending pin number.
 * @param state    The state to set (LED_STATE_ON or LED_STATE_OFF).
 */
void SetLEDState(uint8_t startPin, uint8_t endPin, LEDState state) {
    for (uint8_t pin = startPin; pin <= endPin; pin++) {
        if (pin >= 1 && pin <= 8) { // Ensure pin is within valid range
            GPIO_WriteToOutputPin(GPIOG, pin, (state == LED_STATE_ON) ? ENABLE : DISABLE);
        }
    }
}
