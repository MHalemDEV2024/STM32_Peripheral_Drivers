
#include "stm32f429xx.h"
#include <stddef.h>


/*************************************************************************************************
 * @fn      		      - USART_SetBaudRate
 *
 * @brief             - Configures the baud rate for the specified USART peripheral.
 * 
 * This function calculates and sets the baud rate for the USART peripheral based on
 * the peripheral clock and desired baud rate. It supports both oversampling by 8 and 16.
 *
 * @param[in]         - pUSARTx: Pointer to USART peripheral register definition structure.
 *                      This identifies which USART instance to configure.
 * @param[in]         - BaudRate: Desired baud rate for communication.
 *
 * @return            - None
 *
 * @Note              - The function checks the USART_CR1_OVER8 bit to determine
 *                      whether the oversampling is 8 or 16, and calculates the 
 *                      baud rate accordingly.
 ****************************************************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    // Variable to hold the APB clock
    uint32_t PCLKx;

    // Variable to hold USARTDIV value
    uint32_t usartdiv;

    // Variables to hold Mantissa and Fraction values
    uint32_t M_part, F_part;

    // Temporary register for BRR value
    uint32_t tempreg = 0;

    // 1. Get the APB clock value into PCLKx based on which USART instance is used
    if (pUSARTx == USART1 || pUSARTx == USART6)
    {
        // USART1 and USART6 are connected to APB2
        PCLKx = RCC_GetPCLK2Value();
    }
    else
    {
        // Other USART peripherals (e.g., USART2, USART3) are connected to APB1
        PCLKx = RCC_GetPCLK1Value();
    }

    // 2. Check the OVER8 bit (oversampling by 8 or 16) in USART_CR1
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        // OVER8 = 1, oversampling by 8
        usartdiv = (25 * PCLKx) / (2 * BaudRate);
    }
    else
    {
        // Oversampling by 16 (default)
        usartdiv = (25 * PCLKx) / (4 * BaudRate);
    }

    // 3. Calculate the Mantissa part (integer part of USARTDIV)
    M_part = usartdiv / 100;

    // 4. Load Mantissa part into the temporary register (BRR[15:4])
    tempreg |= M_part << 4;

    // 5. Extract the Fractional part
    F_part = usartdiv - (M_part * 100);

    // 6. Adjust Fractional part based on oversampling mode
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        // OVER8 = 1, fractional part is out of 8
        F_part = (((F_part * 8) + 50) / 100) & 0x07; // Round and mask to 3 bits
    }
    else
    {
        // Oversampling by 16, fractional part is out of 16
        F_part = (((F_part * 16) + 50) / 100) & 0x0F; // Round and mask to 4 bits
    }

    // 7. Load Fractional part into the temporary register (BRR[3:0])
    tempreg |= F_part;

    // 8. Write the final value to the BRR register
    pUSARTx->BRR = tempreg;
}


/************************************************************************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes the given USART peripheral with the specified configuration.
 * 
 * This function configures various settings of the USART peripheral such as 
 * mode (TX/RX), baud rate, word length, parity control, stop bits, and hardware flow control.
 *
 * @param[in]         - pUSARTHandle: Pointer to a USART handle structure that contains
 *                      the configuration settings for the USART peripheral.
 *
 * @return            - None
 *
 * @Note              - Ensure that the clock for the USART peripheral is enabled before using this function.
 **********************************************************************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    // Temporary variable to hold register configurations
    uint32_t tempreg = 0;

/******************************** Configuration of CR1 ******************************************/

    // 1. Enable the clock for the given USART peripheral
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

    // 2. Enable USART Tx and/or Rx engines based on the USART_Mode configuration
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        // Enable the Receiver bit field (RE)
        tempreg |= (1 << USART_CR1_RE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        // Enable the Transmitter bit field (TE)
        tempreg |= (1 << USART_CR1_TE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        // Enable both Transmitter and Receiver bit fields (TE and RE)
        tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }

    // 3. Configure the Word length (8 or 9 bits)
    tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    // 4. Configure the Parity control
    if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        // Enable parity control (PCE)
        tempreg |= (1 << USART_CR1_PCE);
        // No need to configure for even parity as it's default
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
    {
        // Enable parity control (PCE) and select odd parity (PS)
        tempreg |= (1 << USART_CR1_PCE);
        tempreg |= (1 << USART_CR1_PS);
    }

    // 5. Program the CR1 register with the above settings
    pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2 ******************************************/

    // Reset the tempreg for CR2 configuration
    tempreg = 0;

    // 6. Configure the number of stop bits
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

    // 7. Program the CR2 register
    pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3 ******************************************/

    // Reset the tempreg for CR3 configuration
    tempreg = 0;

    // 8. Configure hardware flow control
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        // Enable CTS flow control (CTSE)
        tempreg |= (1 << USART_CR3_CTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        // Enable RTS flow control (RTSE)
        tempreg |= (1 << USART_CR3_RTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        // Enable both CTS and RTS flow control (CTSE and RTSE)
        tempreg |= (1 << USART_CR3_CTSE);
        tempreg |= (1 << USART_CR3_RTSE);
    }

    // 9. Program the CR3 register
    pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR (Baud Rate Register) ******************************************/

    // 10. Configure the baud rate using the helper function
    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}





/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Enables or disables the specified USART peripheral.
 *
 * This function allows for the control of the USART peripheral by enabling 
 * or disabling it. It modifies the CR1 register to either set or clear the 
 * USART enable bit based on the command provided.
 *
 * @param[in]         - pUSARTx: Pointer to the USART peripheral's register 
 *                      definition structure.
 *
 * @param[in]         - Cmd: Command to enable or disable the USART. 
 *                      It can take the following values:
 *                      - ENABLE: To enable the USART peripheral.
 *                      - DISABLE: To disable the USART peripheral.
 *
 * @return            - None
 *
 * @Note              - Ensure that the USART peripheral is properly initialized 
 *                      before calling this function.
 *********************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
    if(Cmd == ENABLE)
    {
        // Set the UE (USART Enable) bit in the CR1 register
        pUSARTx->CR1 |= (1 << 13);
    }
    else
    {
        // Clear the UE (USART Enable) bit in the CR1 register
        pUSARTx->CR1 &= ~(1 << 13);
    }
}


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Controls the clock for a specified USART peripheral.
 *
 * This function enables or disables the clock for the specified USART 
 * peripheral based on the provided command. When enabling, it sets 
 * the relevant clock enable bit in the RCC. When disabling, it clears 
 * the corresponding clock enable bit.
 *
 * @param[in]         - pUSARTx: Pointer to the USART peripheral's 
 *                      register definition structure. Valid options include:
 *                      - USART1
 *                      - USART2
 *                      - USART3
 *                      - UART4
 *                      - UART5
 *                      - USART6
 *                      - UART7
 *                      - UART8
 *
 * @param[in]         - EnorDi: Command to enable or disable the clock. 
 *                      It can take the following values:
 *                      - ENABLE: To enable the clock for the specified USART.
 *                      - DISABLE: To disable the clock for the specified USART.
 *
 * @return            - None
 *
 * @Note              - Ensure the relevant clock control macros are defined 
 *                      for each USART peripheral.
 *
 * @TODO              - Implement the functionality to disable the clock 
 *                      for the specified USART. This may involve 
 *                      adding code to clear the clock enable bits 
 *                      for each USART peripheral in the respective 
 *                      clock control register.
 *********************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        // Enable clock for the specified USART
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();  // Enable clock for USART1
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();  // Enable clock for USART2
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_EN();  // Enable clock for USART3
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_EN();   // Enable clock for UART4
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_EN();   // Enable clock for UART5
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_EN();  // Enable clock for USART6
        }
        else if (pUSARTx == UART7)
        {
            UART7_PCLK_EN();   // Enable clock for UART7
        }
        else if (pUSARTx == UART8)
        {
            UART8_PCLK_EN();   // Enable clock for UART8
        }
    }
    else // EnorDi == DISABLE
    {
			
        // Disable clock for the specified USART
        if (pUSARTx == USART1)
        {
            USART1_PCLK_DI();  // Disable clock for USART1
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_DI();  // Disable clock for USART2
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_DI();  // Disable clock for USART3
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_DI();   // Disable clock for UART4
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_DI();   // Disable clock for UART5
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_DI();  // Disable clock for USART6
        }
        else if (pUSARTx == UART7)
        {
            UART7_PCLK_DI();   // Disable clock for UART7
        }
        else if (pUSARTx == UART8)
        {
            UART8_PCLK_DI();   // Disable clock for UART8
        }
    }
}


/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Checks the status of a specific USART flag.
 *
 * This function reads the status register (SR) of the specified USART 
 * peripheral to determine the current state of a given flag. 
 * It returns a status indicating whether the flag is set or reset.
 *
 * @param[in]         - pUSARTx: Pointer to the USART peripheral's 
 *                      register definition structure. Valid options include:
 *                      - USART1
 *                      - USART2
 *                      - USART3
 *                      - UART4
 *                      - UART5
 *                      - USART6
 *                      - UART7
 *                      - UART8
 *
 * @param[in]         - StatusFlagName: The specific flag to check. 
 *                      This should be one of the USART status flag constants.
 *
 * @return            - Returns SET if the specified flag is set, 
 *                      otherwise returns RESET.
 *
 * @Note              - Ensure that the relevant flag constants are defined 
 *                      for the USART peripheral.
 *********************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    // Check if the specified flag is set in the status register
    if (pUSARTx->SR & StatusFlagName)
    {
        return SET;  // Flag is set
    }

    return RESET;  // Flag is not set
}




/*********************************************************************
 * @fn      		  - USART_Config
 *
 * @brief             - Configures the specified USART interface with custom settings.
 *
 * This function sets up the USART peripheral based on the input parameters, 
 * such as baud rate, hardware flow control, mode, stop bits, word length, 
 * and parity control. It supports configuring any USART peripheral (e.g., 
 * USART1, USART2, etc.) dynamically at runtime.
 *
 * @param[in]         - pUSARTx: Pointer to the USART peripheral (e.g., USART1, USART2).
 *
 * @param[in]         - baud_rate: Baud rate to be configured for the USART.
 *
 * @param[in]         - hw_flow_control: Hardware flow control configuration 
 *                      (e.g., USART_HW_FLOW_CTRL_NONE, USART_HW_FLOW_CTRL_RTS_CTS).
 *
 * @param[in]         - mode: Specifies the mode of operation for the USART 
 *                      (e.g., USART_MODE_ONLY_TX, USART_MODE_ONLY_RX, USART_MODE_TXRX).
 *
 * @param[in]         - stop_bits: Specifies the number of stop bits 
 *                      (e.g., USART_STOPBITS_1, USART_STOPBITS_2).
 *
 * @param[in]         - word_length: Specifies the word length (e.g., USART_WORDLEN_8BITS, USART_WORDLEN_9BITS).
 *
 * @param[in]         - parity_control: Specifies the parity control 
 *                      (e.g., USART_PARITY_DISABLE, USART_PARITY_EVEN, USART_PARITY_ODD).
 *
 * @return            - None
 *
 * @Note              - Ensure the clock for the respective USART is enabled before 
 *                      calling this function. The function must be called after 
 *                      peripheral reset and before data transmission or reception.
 *********************************************************************/
// Define the USART handle
USART_Handle_t usart_handle;

void USART_Config(USART_RegDef_t *pUSARTx, uint32_t baud_rate, uint8_t hw_flow_control, uint8_t mode, uint8_t stop_bits, uint8_t word_length, uint8_t parity_control) {


    // Assign the USART peripheral base address
    usart_handle.pUSARTx = pUSARTx;

    // Configure the USART parameters dynamically based on input arguments
    usart_handle.USART_Config.USART_Baud = baud_rate;
    usart_handle.USART_Config.USART_HWFlowControl = hw_flow_control;
    usart_handle.USART_Config.USART_Mode = mode;
    usart_handle.USART_Config.USART_NoOfStopBits = stop_bits;
    usart_handle.USART_Config.USART_WordLength = word_length;
    usart_handle.USART_Config.USART_ParityControl = parity_control;

    // Initialize the USART with the configured parameters
    USART_Init(&usart_handle);
}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Transmits data over the specified USART interface.
 *
 * This function sends a specified length of data through the USART 
 * peripheral. It handles both 8-bit and 9-bit data formats based on 
 * the configuration settings. The function waits for the transmit 
 * data register to be empty before sending each byte and checks for 
 * the transmission complete flag to ensure the entire buffer has been 
 * transmitted before returning.
 *
 * @param[in]         - pUSARTHandle: Pointer to the USART handle structure
 *                      containing configuration and peripheral information.
 *
 * @param[in]         - pTxBuffer: Pointer to the data buffer to be transmitted.
 *
 * @param[in]         - Len: Number of bytes to be transmitted from the buffer.
 *
 * @return            - None
 *
 * @Note              - Ensure that the USART peripheral is properly 
 *                      initialized before calling this function.
 *********************************************************************/


void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint16_t *pdata;

    // Loop over until "Len" number of bytes are transferred
    for (uint32_t i = 0; i < Len; i++)
    {
        // Wait until TXE flag is set in the SR
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

        // Check the USART_WordLength for 9BIT or 8BIT in a frame
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // Load the DR with 2 bytes, masking bits other than first 9 bits
            pdata = (uint16_t*)pTxBuffer;
            pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

            // Check for USART_ParityControl
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used in this transfer, so 9 bits of user data will be sent
                // Increment pTxBuffer twice for the 9-bit data
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                // Parity bit is used in this transfer, so 8 bits of user data will be sent
                // The 9th bit will be replaced by parity bit by the hardware
                pTxBuffer++;
            }
        }
        else
        {
            // This is an 8-bit data transfer
            pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

            // Increment the buffer address for the next byte
            pTxBuffer++;
        }
    }

    // Wait till TC flag is set in the SR
    while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receives data from the specified USART interface.
 *
 * This function reads a specified number of bytes from the USART 
 * peripheral into the provided buffer. It handles both 8-bit and 
 * 9-bit data formats based on the configuration settings. The function
 * waits for the receive data register to be not empty before reading 
 * each byte.
 *
 * @param[in]         - pUSARTHandle: Pointer to the USART handle structure
 *                      containing configuration and peripheral information.
 *
 * @param[in]         - pRxBuffer: Pointer to the buffer where received data 
 *                      will be stored.
 *
 * @param[in]         - Len: Number of bytes to be received into the buffer.
 *
 * @return            - None
 *
 * @Note              - Ensure that the USART peripheral is properly 
 *                      initialized before calling this function.
 *********************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Loop over until "Len" number of bytes are received
    for (uint32_t i = 0; i < Len; i++)
    {
        // Wait until RXNE flag is set in the SR
        while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

        // Check the USART_WordLength to decide whether we are going to receive 
        // 9 bits of data in a frame or 8 bits
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
        {
            // We are going to receive 9-bit data in a frame

            // Check if we are using USART_ParityControl or not
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used, so all 9 bits will be of user data
                // Read only first 9 bits, so mask the DR with 0x01FF
                *((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

                // Increment the pRxBuffer two times for 9-bit data
                pRxBuffer++;
                pRxBuffer++;
            }
            else
            {
                // Parity is used, so 8 bits will be user data and 1 bit is parity
                *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
                pRxBuffer++;
            }
        }
        else
        {
            // We are going to receive 8-bit data in a frame

            // Check if we are using USART_ParityControl or not
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                // No parity is used, so all 8 bits will be user data
                // Read 8 bits from DR
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
            }
            else
            {
                // Parity is used, so 7 bits will be user data and 1 bit is parity
                // Read only 7 bits, hence mask the DR with 0x7F
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
            }

            // Increment the pRxBuffer for the next byte
            pRxBuffer++;
        }
    }
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Initiates the transmission of data over USART
 *                      using interrupt-driven methods.
 *
 * This function sets up the USART for transmitting a specified length 
 * of data from the provided buffer using interrupts. It first checks 
 * if the USART is currently busy in transmission. If not, it updates 
 * the transmission state, sets the length and buffer, and enables 
 * the necessary transmission interrupts.
 *
 * @param[in]         - pUSARTHandle: Pointer to the USART handle structure 
 *                      containing configuration and peripheral information.
 *
 * @param[in]         - pTxBuffer: Pointer to the buffer containing data to 
 *                      be transmitted.
 *
 * @param[in]         - Len: Number of bytes to be transmitted from the buffer.
 *
 * @return            - uint8_t: The current state of the transmission 
 *                      (e.g., USART_BUSY_IN_TX if busy, otherwise 
 *                      previous state).
 *
 * @Note              - The interrupt handlers for TXE and TC must be 
 *                      properly implemented to manage the transmission 
 *                      process.
 *********************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    // Get the current transmission state
    uint8_t txstate = pUSARTHandle->TxBusyState;

    // Check if USART is not busy in transmission
    if (txstate != USART_BUSY_IN_TX)
    {
        // Set up the transmission parameters
        pUSARTHandle->TxLen = Len;              // Set the length of data to be transmitted
        pUSARTHandle->pTxBuffer = pTxBuffer;    // Set the transmission buffer
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX; // Mark as busy in transmission

        // Enable interrupt for TXE (Transmit Data Register Empty)
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

        // Enable interrupt for TC (Transmission Complete)
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    return txstate; // Return the current transmission state
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Initiates the reception of data over USART
 *                      using interrupt-driven methods.
 *
 * This function sets up the USART for receiving a specified length 
 * of data into the provided buffer using interrupts. It first checks 
 * if the USART is currently busy in reception. If not, it updates 
 * the reception state, sets the length and buffer, and enables 
 * the necessary reception interrupts.
 *
 * @param[in]         - pUSARTHandle: Pointer to the USART handle structure 
 *                      containing configuration and peripheral information.
 *
 * @param[in]         - pRxBuffer: Pointer to the buffer where received data 
 *                      will be stored.
 *
 * @param[in]         - Len: Number of bytes to be received into the buffer.
 *
 * @return            - uint8_t: The current state of the reception 
 *                      (e.g., USART_BUSY_IN_RX if busy, otherwise 
 *                      previous state).
 *
 * @Note              - The interrupt handler for RXNE must be properly 
 *                      implemented to manage the reception process.
 *********************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Get the current reception state
    uint8_t rxstate = pUSARTHandle->RxBusyState;

    // Check if USART is not busy in reception
    if (rxstate != USART_BUSY_IN_RX)
    {
        // Set up the reception parameters
        pUSARTHandle->RxLen = Len;              // Set the length of data to be received
        pUSARTHandle->pRxBuffer = pRxBuffer;    // Set the reception buffer
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX; // Mark as busy in reception

        // Clear the data register (optional, ensures the first read is valid)
        (void)pUSARTHandle->pUSARTx->DR; 

        // Enable interrupt for RXNE (Receive Data Register Not Empty)
        pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    return rxstate; // Return the current reception state
}


/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Clears specified status flags in the USART status register.
 *
 * This function clears the given status flag(s) in the USART's status 
 * register by performing a bitwise AND operation with the complement 
 * of the specified flag. It is applicable for specific flags like 
 * USART_CTS_FLAG, USART_LBD_FLAG, USART_TC_FLAG, and others.
 *
 * @param[in]         - pUSARTx: Pointer to the USART peripheral's register 
 *                      definition structure.
 *
 * @param[in]         - StatusFlagName: The status flag(s) to be cleared. 
 *                      This should be one of the predefined status flag 
 *                      constants such as USART_CTS_FLAG, USART_LBD_FLAG, 
 *                      USART_TC_FLAG, etc.
 *
 * @return            - void: This function does not return a value.
 *
 * @Note              - This function is designed to be used with status 
 *                      flags that can be cleared by writing to the SR 
 *                      register.
 *********************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    // Clear the specified status flag by masking it out in the SR register
    pUSARTx->SR &= ~(StatusFlagName);
}


/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}



/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
*/
 
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}


