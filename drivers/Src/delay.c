#include "delay.h"

// Dynamic delay function
/*
 * This function generates a delay based on the input time in seconds.
 *
 * Clock Speed: 16 MHz or based on actual clock configuration.
 * Each iteration of the loop takes approximately 13 clock cycles (depends on the specific architecture and optimization).
 * The delay time is calculated as: delay_time (in seconds) * (Clock Speed / Cycles Per Iteration).
 *
 * Adjust the constant if you are using a different clock speed or find the exact cycle count per loop iteration.
 */

void delay(uint32_t seconds) {
    uint32_t clock_speed = 16000000; // Replace with actual system clock if known
    uint32_t cycles_per_iteration = 13; // Estimate the cycles per loop iteration (adjust based on actual measurement)
    
    // Calculate number of iterations needed
    uint32_t count = (seconds * clock_speed) / cycles_per_iteration;
    
    // Perform the delay
    for (uint32_t i = 0; i < count; i++) {
        __asm("NOP"); // No operation, just burn cycles
    }
}
