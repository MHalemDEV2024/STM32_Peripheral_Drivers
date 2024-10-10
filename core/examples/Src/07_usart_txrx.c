#include "stm32f429xx.h"
#include <string.h>
#include <ctype.h>  // For tolower() function
#include "delay.h"

// Constants
#define RX_BUF_SIZE 1024  // Size of the receive buffer
#define LED_PIN GPIO_PIN_NO_13  // Define LED pin for clarity

// Message buffer for user prompt
const char msg[] = "Please type 'on' and press Enter to turn on the LED:\n\r";

// Global variables
uint8_t g_data = 0;  // Variable to store received data
char rx_buf[RX_BUF_SIZE];  // Buffer to store input characters
int buffer_index = 0;  // Index to track position in buffer

// Function prototypes
void USART_GPIO_Init(void);
void USART_Config_Init(void);
void check_input_and_toggle_led(char *input_buffer);

/***********************************************************************************************
 * @fn      		  - main
 * 
 * @brief             - Main function initializes peripherals and processes user input.
 * 
 * @param[in]         - None
 * 
 * @return            - None
 *************************************************************************************************/
int main(void) {
    // Enable clock for GPIOG
    GPIO_PeriClockControl(GPIOG, ENABLE);

    // Initialize LED pin
    GPIO_PinInit(GPIOG, LED_PIN, GPIO_MODE_OUT, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD, 0);

    // Initialize USART GPIO and configuration
    USART_GPIO_Init();
    USART_Config_Init();

    // Send initial message to user
    USART_SendData(&usart_handle, (uint8_t *)msg, strlen(msg));
    delay(1); // Simple delay

    // Main loop to handle user input
    while (1) {
        // Polling mechanism for receiving data
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {  // Check if data is available in the receive buffer
            // Receive data
            USART_ReceiveData(&usart_handle, &g_data, 1);  // Read 1 byte of data from USART

            // Check for newline characters to process input
            if (g_data == '\n' || g_data == '\r') {
                check_input_and_toggle_led(rx_buf);  // Process the received input
                buffer_index = 0;  // Reset buffer index after processing
                memset(rx_buf, 0, sizeof(rx_buf));  // Clear the buffer
            } else {
                // Add character to input buffer while preventing overflow
                if (buffer_index < RX_BUF_SIZE - 1) {  
                    rx_buf[buffer_index++] = g_data;
                    rx_buf[buffer_index] = '\0';  // Null-terminate the string
                } else {
                    // Handle buffer overflow
                    const char msg_overflow[] = "\nInput buffer overflow. Please try again.\n\r";
                    USART_SendData(&usart_handle, (uint8_t *)msg_overflow, strlen(msg_overflow));
                    buffer_index = 0;  // Reset buffer index
                    memset(rx_buf, 0, sizeof(rx_buf));  // Clear the buffer
                }
            }
        }
    }
}

/***********************************************************************************************
 * @fn      		  - check_input_and_toggle_led
 * 
 * @brief             - Handles input commands to toggle the LED state.
 * 
 * This function checks the input string for valid commands to either turn the LED ON or OFF.
 * It accepts "on", "ON", "1" to turn the LED on and "off", "OFF", "0" to turn the LED off.
 * It provides feedback for incomplete commands and resets the buffer after processing.
 * 
 * @param[in] input_buffer - Pointer to the input character buffer.
 * @return    None
 *************************************************************************************************/
void check_input_and_toggle_led(char *input_buffer) {
    // Convert input to lowercase for case-insensitive comparison
    for (int i = 0; input_buffer[i] != '\0'; i++) {
        input_buffer[i] = tolower(input_buffer[i]);
    }

    // Check for valid "on" or "1" command
    if (strcmp(input_buffer, "on") == 0 || strcmp(input_buffer, "1") == 0) {
        GPIO_WriteToOutputPin(GPIOG, LED_PIN, GPIO_PIN_SET);  // Turn ON LED
        const char msg_on[] = "\nLED is ON\n\r";
        USART_SendData(&usart_handle, (uint8_t *)msg_on, strlen(msg_on));  // Send confirmation message
    } 
    // Check for valid "off" or "0" command
    else if (strcmp(input_buffer, "off") == 0 || strcmp(input_buffer, "0") == 0) {
        GPIO_WriteToOutputPin(GPIOG, LED_PIN, GPIO_PIN_RESET);  // Turn OFF LED
        const char msg_off[] = "\nLED is OFF\n\r";
        USART_SendData(&usart_handle, (uint8_t *)msg_off, strlen(msg_off));  // Send confirmation message
    } 
    // Check for incomplete commands
    else if (strncmp(input_buffer, "o", 1) == 0) {
        const char msg_partial_on[] = "\nIncomplete command. Did you mean 'on'?\n\r";
        USART_SendData(&usart_handle, (uint8_t *)msg_partial_on, strlen(msg_partial_on));
    } 
    else if (strncmp(input_buffer, "f", 1) == 0) {
        const char msg_partial_off[] = "\nIncomplete command. Did you mean 'off'?\n\r";
        USART_SendData(&usart_handle, (uint8_t *)msg_partial_off, strlen(msg_partial_off));
    }
    // Invalid input
    else {
        const char msg_invalid[] = "\nInvalid command. Please type 'on', 'off', '1', or '0'.\n\r";
        USART_SendData(&usart_handle, (uint8_t *)msg_invalid, strlen(msg_invalid));  // Send invalid input message
    }

    // Reset the input buffer and index after processing the command
    buffer_index = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
}

/***********************************************************************************************
 * @fn      		  - USART_GPIO_Init
 * 
 * @brief             - Initializes the GPIO pins for USART1 (PA9 as TX, PA10 as RX).
 * 
 * @param[in]         - None
 * 
 * @return            - None
 *************************************************************************************************/
void USART_GPIO_Init(void) {
    // Enable GPIOA peripheral clock
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Initialize PA9 (USART1_TX) with alternate function AF7
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_9, GPIO_MODE_ALTFN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD, 7);

    // Initialize PA10 (USART1_RX) with alternate function AF7
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_10, GPIO_MODE_ALTFN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD, 7);
}

/***********************************************************************************************
 * @fn      		  - USART_Config_Init
 * 
 * @brief             - Configures USART1 with standard settings (115200 baud, TX/RX mode).
 * 
 * @param[in]         - None
 * 
 * @return            - None
 *************************************************************************************************/
void USART_Config_Init(void) {
    // Set up the USART handle structure with configuration parameters
    USART_Config(USART1, USART_STD_BAUD_115200, USART_HW_FLOW_CTRL_NONE, 
                 USART_MODE_TXRX, USART_STOPBITS_1, USART_WORDLEN_8BITS, 
                 USART_PARITY_DISABLE);

    // Enable USART1 peripheral
    USART_PeripheralControl(USART1, ENABLE);
}
