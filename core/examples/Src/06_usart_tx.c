#include "stm32f429xx.h"
#include <string.h>
#include "delay.h"

// Message buffers
const char msg[] = "USART Initialized!\n\r";
const char msg1[] = "Hello, world!\n\r";

// Function prototypes
void USART_GPIO_Init(void);
void USART_Config_Init(void);

// Main function
int main(void) {
    // Initialize the GPIO pins for USART
    USART_GPIO_Init();

    // Initialize the USART peripheral
    USART_Config_Init();


    // Send initialization message
    USART_SendData(&usart_handle, (uint8_t *)msg, strlen(msg));
    delay(1); // Simple delay for ensuring transmission

    // Main loop: continuously send another message
    while (1) {
        USART_SendData(&usart_handle, (uint8_t *)msg1, strlen(msg1));
        delay(1); // Delay between transmissions
    }

    return 0; // This line may not be reached in embedded systems, but included for completeness
}

/*********************************************************************
 * @fn      		  - USART_GPIO_Init
 * 
 * @brief             - Initializes the GPIO pins for USART1 (PA9 as TX, PA10 as RX).
 * 
 * This function configures GPIOA pins 9 and 10 for USART1 transmit and 
 * receive functionality using alternate function mode. It ensures that 
 * the pins operate at a fast speed and are configured for push-pull 
 * output type with no pull-up/pull-down resistors.
 * 
 * @param[in]         - None
 * 
 * @return            - None
 * 
 *********************************************************************/
void USART_GPIO_Init(void) {
    // Enable GPIOA peripheral clock
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // Initialize PA9 (USART1_TX) with alternate function AF7
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_9, GPIO_MODE_ALTFN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD, 7);

    // Initialize PA10 (USART1_RX) with alternate function AF7
    GPIO_PinInit(GPIOA, GPIO_PIN_NO_10, GPIO_MODE_ALTFN, GPIO_SPEED_FAST, GPIO_OP_TYPE_PP, GPIO_NO_PUPD, 7);
}

/*********************************************************************
 * @fn      		  - USART_Config_Init
 * 
 * @brief             - Configures the USART1 peripheral with standard settings.
 * 
 * This function sets up USART1 with a baud rate of 115200, no hardware 
 * flow control, transmit-only mode, 1 stop bit, 8-bit word length, 
 * and parity disabled. It also enables the USART1 peripheral.
 * 
 * @param[in]         - None
 * 
 * @return            - None
 * 
 *********************************************************************/
void USART_Config_Init(void) {
    // Set up the USART handle structure with configuration parameters
    USART_Config(USART1, USART_STD_BAUD_115200, USART_HW_FLOW_CTRL_NONE, 
                 USART_MODE_ONLY_TX, USART_STOPBITS_1, USART_WORDLEN_8BITS, 
                 USART_PARITY_DISABLE);

    // Enable USART1 peripheral
    USART_PeripheralControl(USART1, ENABLE);
}
