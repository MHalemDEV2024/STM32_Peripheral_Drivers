/*
 * 004_led_button.c
 *
 *  Created on: Sep 17, 2024
 *  Author: Mohamed Saeed
 *  Description: This program controls the shifting of LEDs on GPIOG pins 1 to 8
 *               based on the input from buttons on GPIOA pins 0 and 2. It utilizes
 *               GPIO interrupts to toggle LED shifting and direction.
 */

#include "stm32f429xx.h"
#include "delay.h"

#define LEFT_SHIFT  0
#define RIGHT_SHIFT 1

typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LEDState;

// Function prototypes
static void System_GPIO_Init(void);
static void GPIO_IRQ_Init(void);
static void ShiftLEDs(uint8_t startPin, uint8_t shiftCount, uint32_t delayTime, uint8_t direction, uint8_t shiftSize);
static void SetLEDState(uint8_t startPin, uint8_t endPin, LEDState state);
static void HandleLEDShift(void);
void EXTI0_IRQHandler(void);
void EXTI2_IRQHandler(void);

// Global variables for LED shifting
volatile uint8_t ledShiftEnabled = 0;       // Flag to enable or disable LED shifting
volatile uint8_t shiftDirection = LEFT_SHIFT;  // Current shift direction

int main(void) {
    System_GPIO_Init();       // Initialize GPIO pins
    GPIO_IRQ_Init();        // Initialize GPIO interrupts

    while (1) {
        HandleLEDShift();  // Continuously handle LED shifting based on current state
    }
}

// Function to initialize GPIO pins
static void System_GPIO_Init(void) {
    // Enable clocks for GPIO ports
    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Configure GPIO pins for LEDs (GPIOG, Pins 1 to 8)
    for (uint8_t i = 1; i <= 8; i++) {
        GPIO_PinInit(GPIOG, i, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    }

    // Configure additional control LEDs (Pins 13 and 14)
    GPIO_PinInit(GPIOG, GPIO_PIN_NO_13, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    GPIO_PinInit(GPIOG, GPIO_PIN_NO_14, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);

    // Configure GPIO pins for buttons (GPIOA, Pins 0 and 2)
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_0, GPIO_MODE_IT_FT, GPIO_SPEED_MEDIUM, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_2, GPIO_MODE_IT_FT, GPIO_SPEED_MEDIUM, GPIO_OP_TYPE_PP, GPIO_NO_PUPD);
}

// Function to initialize GPIO interrupts
static void GPIO_IRQ_Init(void) {
    // Configure EXTI0 interrupt (GPIOA Pin 0) with lowest priority
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    // Configure EXTI2 interrupt (GPIOA Pin 2) with lowest priority
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);
}

// Function to handle LED shifting based on current state and direction
static void HandleLEDShift(void) {
    if (ledShiftEnabled) {
        GPIO_WriteToOutputPin(GPIOG, GPIO_PIN_NO_14, DISABLE);  // Disable control LED

        // Shift LEDs based on the current direction
        ShiftLEDs(1, 8, 1, shiftDirection, (shiftDirection == LEFT_SHIFT) ? 1 : 2);
    } else {
        // Turn off all LEDs and ensure control LED is enabled
        SetLEDState(1, 8, LED_OFF);
        GPIO_WriteToOutputPin(GPIOG, GPIO_PIN_NO_14, ENABLE);
    }
}

// Function to shift LEDs left or right
static void ShiftLEDs(uint8_t startPin, uint8_t shiftCount, uint32_t delayTime, uint8_t direction, uint8_t shiftSize) {
    // Validate parameters
    if (startPin < 1 || startPin > 8 || shiftSize < 1 || shiftSize > shiftCount) return;

    if (direction == LEFT_SHIFT) {
        // Shift LEDs left
        for (uint8_t j = 0; j <= shiftCount - shiftSize; j += shiftSize) {
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_ON);
            delay(delayTime);
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_OFF);

            // Exit if shifting is disabled or direction changed
            if (!ledShiftEnabled || shiftDirection != LEFT_SHIFT) return;
        }
    } else if (direction == RIGHT_SHIFT) {
        // Shift LEDs right
        for (int8_t j = shiftCount - shiftSize; j >= 0; j -= shiftSize) {
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_ON);
            delay(delayTime);
            SetLEDState(startPin + j, startPin + j + shiftSize - 1, LED_OFF);

            // Exit if shifting is disabled or direction changed
            if (!ledShiftEnabled || shiftDirection != RIGHT_SHIFT) return;
        }
    }
}

// Function to set the state of LEDs in a range
static void SetLEDState(uint8_t startPin, uint8_t endPin, LEDState state) {
    for (uint8_t pin = startPin; pin <= endPin; pin++) {
        if (pin >= 1 && pin <= 8) {
            GPIO_WriteToOutputPin(GPIOG, pin, (state == LED_ON) ? ENABLE : DISABLE);
        }
    }
}

// EXTI0 Interrupt Service Routine
void EXTI0_IRQHandler(void) {
    GPIO_IRQHandling(GPIO_PIN_NO_0);  // Clear the interrupt pending bit

    // Toggle control LED on GPIO pin 14
    GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_14);

    // Toggle LED shifting state
    ledShiftEnabled = !ledShiftEnabled;
    if (!ledShiftEnabled) {
        SetLEDState(1, 8, LED_OFF);  // Ensure all LEDs are off
    }
}

// EXTI2 Interrupt Service Routine
void EXTI2_IRQHandler(void) {
    GPIO_IRQHandling(GPIO_PIN_NO_2);  // Clear the interrupt pending bit

    // Toggle control LED on GPIO pin 13
    GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13);

    // Toggle the direction of LED shifting
    shiftDirection = (shiftDirection == LEFT_SHIFT) ? RIGHT_SHIFT : LEFT_SHIFT;

    // Reset all LEDs
    SetLEDState(1, 8, LED_OFF);
}
