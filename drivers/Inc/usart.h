#ifndef USART_H_
#define USART_H_

#include "stm32f429xx.h"

/*
 * Peripheral register definition structure for USART
 *
 * This structure defines the memory-mapped registers for the USART peripheral.
 * Each register is defined as a volatile 32-bit unsigned integer (__vo uint32_t),
 * ensuring the compiler does not optimize away accesses to these hardware registers.
 */
typedef struct
{
    __vo uint32_t SR;     /*!< Status Register,                     Address offset: 0x00 */
    __vo uint32_t DR;     /*!< Data Register,                       Address offset: 0x04 */
    __vo uint32_t BRR;    /*!< Baud Rate Register,                  Address offset: 0x08 */
    __vo uint32_t CR1;    /*!< Control Register 1,                  Address offset: 0x0C */
    __vo uint32_t CR2;    /*!< Control Register 2,                  Address offset: 0x10 */
    __vo uint32_t CR3;    /*!< Control Register 3,                  Address offset: 0x14 */
    __vo uint32_t GTPR;   /*!< Guard Time and Prescaler Register,   Address offset: 0x18 */
} USART_RegDef_t;

/*
 * Configuration structure for USART peripheral
 * 
 * This structure holds the configuration settings for a USART peripheral.
 * It defines the mode, baud rate, stop bits, word length, parity control, 
 * and hardware flow control options that can be configured.
 */
typedef struct
{
    uint8_t USART_Mode;           /*!< Specifies the mode of operation: TX only, RX only, or both TX and RX. */
    uint32_t USART_Baud;          /*!< Specifies the baud rate for communication (e.g., 9600, 115200, etc.). */
    uint8_t USART_NoOfStopBits;   /*!< Specifies the number of stop bits: 1, 0.5, 2, or 1.5 stop bits. */
    uint8_t USART_WordLength;     /*!< Specifies the word length for data transmission: 8 or 9 bits. */
    uint8_t USART_ParityControl;  /*!< Specifies parity control: odd, even, or none (disabled). */
    uint8_t USART_HWFlowControl;  /*!< Specifies hardware flow control: none, CTS, RTS, or both CTS and RTS. */
} USART_Config_t;


/*
 * Handle structure for USART peripheral
 *
 * This structure is used to handle and manage the USART peripheral,
 * holding its configuration, status, and data buffers for transmission
 * and reception.
 */
typedef struct
{
    USART_RegDef_t *pUSARTx;         /*!< Pointer to the base address of the USART peripheral */
    USART_Config_t USART_Config;     /*!< USART configuration settings (mode, baud rate, etc.) */
    uint8_t *pTxBuffer;              /*!< Pointer to the transmission (Tx) buffer */
    uint8_t *pRxBuffer;              /*!< Pointer to the reception (Rx) buffer */
    uint32_t TxLen;                  /*!< Length of the transmission (Tx) buffer */
    uint32_t RxLen;                  /*!< Length of the reception (Rx) buffer */
    uint8_t TxBusyState;             /*!< Transmission busy state: indicates if Tx is in progress */
    uint8_t RxBusyState;             /*!< Reception busy state: indicates if Rx is in progress */
} USART_Handle_t;



/**
 * @brief Base addresses for USART peripherals
 *        Define pointers to USART and UART registers, mapped to their respective base addresses.
 */

/* USART Peripheral Base Addresses */
#define USART1      ((USART_RegDef_t*)USART1_BASEADDR)  /*!< Base address for USART1 */
#define USART2      ((USART_RegDef_t*)USART2_BASEADDR)  /*!< Base address for USART2 */
#define USART3      ((USART_RegDef_t*)USART3_BASEADDR)  /*!< Base address for USART3 */
#define USART6      ((USART_RegDef_t*)USART6_BASEADDR)  /*!< Base address for USART6 */

/* UART Peripheral Base Addresses */
#define UART4       ((USART_RegDef_t*)UART4_BASEADDR)   /*!< Base address for UART4 */
#define UART5       ((USART_RegDef_t*)UART5_BASEADDR)   /*!< Base address for UART5 */
#define UART7       ((USART_RegDef_t*)UART7_BASEADDR)   /*!< Base address for UART7 */
#define UART8       ((USART_RegDef_t*)UART8_BASEADDR)   /*!< Base address for UART8 */



/******************************************************************************/ 
/*                                USART Macros                                */ 
/******************************************************************************/ 

/**
 * @brief USART Mode options
 *        Defines the available modes for USART communication.
 */
#define USART_MODE_ONLY_TX       0  /*!< USART in Transmit-only mode */
#define USART_MODE_ONLY_RX       1  /*!< USART in Receive-only mode */
#define USART_MODE_TXRX          2  /*!< USART in both Transmit and Receive mode */

/**
 * @brief Standard Baud Rates
 *        Common baud rate settings for USART communication.
 */
#define USART_STD_BAUD_1200      1200      /*!< Baud rate: 1200 bps */
#define USART_STD_BAUD_2400      2400      /*!< Baud rate: 2400 bps */
#define USART_STD_BAUD_9600      9600      /*!< Baud rate: 9600 bps */
#define USART_STD_BAUD_19200     19200     /*!< Baud rate: 19200 bps */
#define USART_STD_BAUD_38400     38400     /*!< Baud rate: 38400 bps */
#define USART_STD_BAUD_57600     57600     /*!< Baud rate: 57600 bps */
#define USART_STD_BAUD_115200    115200    /*!< Baud rate: 115200 bps */
#define USART_STD_BAUD_230400    230400    /*!< Baud rate: 230400 bps */
#define USART_STD_BAUD_460800    460800    /*!< Baud rate: 460800 bps */
#define USART_STD_BAUD_921600    921600    /*!< Baud rate: 921600 bps */
#define USART_STD_BAUD_2M        2000000   /*!< Baud rate: 2 Mbps */
#define USART_STD_BAUD_3M        3000000   /*!< Baud rate: 3 Mbps */

/**
 * @brief Parity Control options
 *        Options for configuring USART parity control.
 */
#define USART_PARITY_DISABLE     0  /*!< No parity bit */
#define USART_PARITY_EN_EVEN     1  /*!< Even parity */
#define USART_PARITY_EN_ODD      2  /*!< Odd parity */

/**
 * @brief Word Length options
 *        Defines the length of data words for USART transmission/reception.
 */
#define USART_WORDLEN_8BITS      0  /*!< 8-bit word length */
#define USART_WORDLEN_9BITS      1  /*!< 9-bit word length */

/**
 * @brief Stop Bits options
 *        Configurable number of stop bits in USART frame.
 */
#define USART_STOPBITS_1         0  /*!< 1 Stop bit */
#define USART_STOPBITS_0_5       1  /*!< 0.5 Stop bit */
#define USART_STOPBITS_2         2  /*!< 2 Stop bits */
#define USART_STOPBITS_1_5       3  /*!< 1.5 Stop bits */

/**
 * @brief Hardware Flow Control options
 *        Control flow of data using CTS/RTS lines.
 */
#define USART_HW_FLOW_CTRL_NONE      0  /*!< No hardware flow control */
#define USART_HW_FLOW_CTRL_CTS       1  /*!< CTS flow control enabled */
#define USART_HW_FLOW_CTRL_RTS       2  /*!< RTS flow control enabled */
#define USART_HW_FLOW_CTRL_CTS_RTS   3  /*!< CTS and RTS flow control enabled */

/**
 * @brief USART Status Flags
 *        Bit positions representing USART status register flags.
 */
#define USART_FLAG_TXE      (1 << USART_SR_TXE)    /*!< Transmit data register empty */
#define USART_FLAG_RXNE     (1 << USART_SR_RXNE)   /*!< Read data register not empty */
#define USART_FLAG_TC       (1 << USART_SR_TC)     /*!< Transmission complete */

/**
 * @brief Application states
 *        Describes the state of the USART during communication.
 */
#define USART_READY         0  /*!< USART is ready */
#define USART_BUSY_IN_RX    1  /*!< USART is busy in receiving */
#define USART_BUSY_IN_TX    2  /*!< USART is busy in transmitting */

/**
 * @brief USART Event macros
 *        Events triggered during USART communication.
 */
#define USART_EVENT_TX_CMPLT   0  /*!< Transmission complete */
#define USART_EVENT_RX_CMPLT   1  /*!< Reception complete */
#define USART_EVENT_IDLE       2  /*!< Idle line detected */
#define USART_EVENT_CTS        3  /*!< CTS flag detected */
#define USART_EVENT_PE         4  /*!< Parity error */
#define USART_ERR_FE           5  /*!< Framing error */
#define USART_ERR_NE           6  /*!< Noise error */
#define USART_ERR_ORE          7  /*!< Overrun error */



/******************************************************************************/ 
/*           Bit Position Definitions for USART Control and Status Registers  */
/******************************************************************************/ 

/**
 * @brief Bit position definitions for USART Control Register 1 (CR1)
 */
#define USART_CR1_SBK      0   /*!< Send Break */
#define USART_CR1_RWU      1   /*!< Receiver wakeup */
#define USART_CR1_RE       2   /*!< Receiver enable */
#define USART_CR1_TE       3   /*!< Transmitter enable */
#define USART_CR1_IDLEIE   4   /*!< IDLE interrupt enable */
#define USART_CR1_RXNEIE   5   /*!< RXNE interrupt enable */
#define USART_CR1_TCIE     6   /*!< Transmission complete interrupt enable */
#define USART_CR1_TXEIE    7   /*!< TXE interrupt enable */
#define USART_CR1_PEIE     8   /*!< Parity error interrupt enable */
#define USART_CR1_PS       9   /*!< Parity selection */
#define USART_CR1_PCE      10  /*!< Parity control enable */
#define USART_CR1_WAKE     11  /*!< Wakeup method */
#define USART_CR1_M        12  /*!< Word length */
#define USART_CR1_UE       13  /*!< USART enable */
#define USART_CR1_OVER8    15  /*!< Oversampling mode */

/**
 * @brief Bit position definitions for USART Control Register 2 (CR2)
 */
#define USART_CR2_ADD      0   /*!< Address of the USART node */
#define USART_CR2_LBDL     5   /*!< LIN break detection length */
#define USART_CR2_LBDIE    6   /*!< LIN break detection interrupt enable */
#define USART_CR2_LBCL     8   /*!< Last bit clock pulse */
#define USART_CR2_CPHA     9   /*!< Clock phase */
#define USART_CR2_CPOL     10  /*!< Clock polarity */
#define USART_CR2_STOP     12  /*!< STOP bits */
#define USART_CR2_LINEN    14  /*!< LIN mode enable */

/**
 * @brief Bit position definitions for USART Control Register 3 (CR3)
 */
#define USART_CR3_EIE      0   /*!< Error interrupt enable */
#define USART_CR3_IREN     1   /*!< IrDA mode enable */
#define USART_CR3_IRLP     2   /*!< IrDA low-power mode */
#define USART_CR3_HDSEL    3   /*!< Half-duplex selection */
#define USART_CR3_NACK     4   /*!< Smartcard NACK enable */
#define USART_CR3_SCEN     5   /*!< Smartcard mode enable */
#define USART_CR3_DMAR     6   /*!< DMA enable receiver */
#define USART_CR3_DMAT     7   /*!< DMA enable transmitter */
#define USART_CR3_RTSE     8   /*!< RTS enable */
#define USART_CR3_CTSE     9   /*!< CTS enable */
#define USART_CR3_CTSIE    10  /*!< CTS interrupt enable */
#define USART_CR3_ONEBIT   11  /*!< One sample bit method enable */

/**
 * @brief Bit position definitions for USART Status Register (SR)
 */
#define USART_SR_PE        0   /*!< Parity error */
#define USART_SR_FE        1   /*!< Framing error */
#define USART_SR_NE        2   /*!< Noise detected flag */
#define USART_SR_ORE       3   /*!< Overrun error */
#define USART_SR_IDLE      4   /*!< IDLE line detected */
#define USART_SR_RXNE      5   /*!< Read data register not empty */
#define USART_SR_TC        6   /*!< Transmission complete */
#define USART_SR_TXE       7   /*!< Transmit data register empty */
#define USART_SR_LBD       8   /*!< LIN break detection flag */
#define USART_SR_CTS       9   /*!< CTS flag */


/******************************************************************************************
 *                              API Prototypes
 ******************************************************************************************/

// Declare usart_handle as extern
extern USART_Handle_t usart_handle;

/*
 * Peripheral Clock setup
 * This function enables or disables the clock for a specified USART peripheral.
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Initialization and De-initialization
 * Initializes the USART peripheral with the provided settings.
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

void USART_Config(USART_RegDef_t *pUSARTx, uint32_t baud_rate, uint8_t hw_flow_control, uint8_t mode, uint8_t stop_bits, uint8_t word_length, uint8_t parity_control);

/*
 * Send and Receive Data
 * Sends a byte of data through the USART peripheral.
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 * Configures the USART interrupt requests.
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Peripheral Control APIs
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif /* USART_H_ */
