#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32f429xx.h"


/******************************************************************************/ 
/*                    RCC Peripheral Register Definition Structure            */
/*                  Based on STM32F429 Reference Manual (RM0090, Table 34)     */
/******************************************************************************/ 

/** 
 * @brief RCC (Reset and Clock Control) Register Definition Structure 
 *        This structure represents the layout of the RCC registers, providing 
 *        direct access to the system clock control, reset control, and clock gating 
 *        functionality of the STM32F429 microcontroller.
 */ 
typedef struct
{
    __vo uint32_t CR;            /*!< RCC clock control register,                          Address offset: 0x00 */ 
    __vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                      Address offset: 0x04 */ 
    __vo uint32_t CFGR;          /*!< RCC clock configuration register,                    Address offset: 0x08 */ 
    __vo uint32_t CIR;           /*!< RCC clock interrupt register,                        Address offset: 0x0C */ 
    __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                  Address offset: 0x10 */ 
    __vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                  Address offset: 0x14 */ 
    __vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                  Address offset: 0x18 */ 
    uint32_t RESERVED0;          /*!< Reserved,                                            Address offset: 0x1C */ 
    __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                  Address offset: 0x20 */ 
    __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                  Address offset: 0x24 */ 
    uint32_t RESERVED1[2];       /*!< Reserved,                                            Address offset: 0x28-0x2C */ 
    __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register,           Address offset: 0x30 */ 
    __vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock enable register,           Address offset: 0x34 */ 
    __vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock enable register,           Address offset: 0x38 */ 
    uint32_t RESERVED2;          /*!< Reserved,                                            Address offset: 0x3C */ 
    __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,           Address offset: 0x40 */ 
    __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,           Address offset: 0x44 */ 
    uint32_t RESERVED3[2];       /*!< Reserved,                                            Address offset: 0x48-0x4C */ 
    __vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode,  Address offset: 0x50 */ 
    __vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode,  Address offset: 0x54 */ 
    __vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode,  Address offset: 0x58 */ 
    uint32_t RESERVED4;          /*!< Reserved,                                            Address offset: 0x5C */ 
    __vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode,  Address offset: 0x60 */ 
    __vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode,  Address offset: 0x64 */ 
    uint32_t RESERVED5[2];       /*!< Reserved,                                            Address offset: 0x68-0x6C */ 
    __vo uint32_t BDCR;          /*!< RCC Backup domain control register,                  Address offset: 0x70 */ 
    __vo uint32_t CSR;           /*!< RCC clock control & status register,                 Address offset: 0x74 */ 
    uint32_t RESERVED6[2];       /*!< Reserved,                                            Address offset: 0x78-0x7C */ 
    __vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,       Address offset: 0x80 */ 
    __vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                   Address offset: 0x84 */ 
    __vo uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                   Address offset: 0x88 */ 
    __vo uint32_t DCKCFGR;       /*!< RCC dedicated clocks configuration register,         Address offset: 0x8C */ 
    __vo uint32_t CKGATENR;      /*!< RCC clocks gated enable register,                    Address offset: 0x90 */ 
    __vo uint32_t DCKCFGR2;      /*!< RCC dedicated clocks configuration register 2,       Address offset: 0x94 */ 
} RCC_RegDef_t;


/******************************************************************************/ 
/*        Peripheral Definitions (Base Addresses Typecast to RegDef_t)         */ 
/******************************************************************************/ 

/** 
 * @brief RCC peripheral definition
 * 
 * This macro maps the base address of the RCC peripheral (RCC_BASEADDR) 
 * to a pointer of type RCC_RegDef_t, which represents the structure of 
 * the RCC register block. This allows for direct access and modification 
 * of the RCC registers using a structured syntax.
 *
 * Example Usage:
 *    - RCC->CR    : Accesses the clock control register.
 *    - RCC->AHB1ENR : Enables/disables the AHB1 peripheral clocks.
 *
 * By using this macro, developers can reference RCC peripheral registers 
 * conveniently as part of the RCC_RegDef_t structure.
 */


#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)      /*!< Pointer to RCC registers */



/******************************************************************************/ 
/*               Peripheral Clock Enable/Disable/Reset Macros                 */
/******************************************************************************/ 

/*
 * @brief Clock Enable Macros for GPIOx Peripherals 
 * 
 * These macros enable the clock for the GPIO peripherals using the AHB1ENR 
 * (AHB1 peripheral clock enable register) in the RCC. Each macro sets the 
 * respective bit for a GPIO port to enable its clock.
 */
#define GPIOA_CLK_EN()    (RCC->AHB1ENR |= (1 << 0))   /*!< Enable clock for GPIOA */
#define GPIOB_CLK_EN()    (RCC->AHB1ENR |= (1 << 1))   /*!< Enable clock for GPIOB */
#define GPIOC_CLK_EN()    (RCC->AHB1ENR |= (1 << 2))   /*!< Enable clock for GPIOC */
#define GPIOD_CLK_EN()    (RCC->AHB1ENR |= (1 << 3))   /*!< Enable clock for GPIOD */
#define GPIOE_CLK_EN()    (RCC->AHB1ENR |= (1 << 4))   /*!< Enable clock for GPIOE */
#define GPIOF_CLK_EN()    (RCC->AHB1ENR |= (1 << 5))   /*!< Enable clock for GPIOF */
#define GPIOG_CLK_EN()    (RCC->AHB1ENR |= (1 << 6))   /*!< Enable clock for GPIOG */
#define GPIOH_CLK_EN()    (RCC->AHB1ENR |= (1 << 7))   /*!< Enable clock for GPIOH */
#define GPIOI_CLK_EN()    (RCC->AHB1ENR |= (1 << 8))   /*!< Enable clock for GPIOI */



/**
 * @brief Clock Enable Macros for USART/UART on APB2 Bus
 *        Enable clock for USART peripherals on APB2 bus.
 */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))   /*!< Enable clock for USART1 */
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << 5))   /*!< Enable clock for USART6 */

/**
 * @brief Clock Enable Macros for USART/UART on APB1 Bus
 *        Enable clock for USART/UART peripherals on APB1 bus.
 */
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))  /*!< Enable clock for USART2 */
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))  /*!< Enable clock for USART3 */
#define UART4_PCLK_EN()         (RCC->APB1ENR |= (1 << 19))  /*!< Enable clock for UART4  */
#define UART5_PCLK_EN()         (RCC->APB1ENR |= (1 << 20))  /*!< Enable clock for UART5  */
#define UART7_PCLK_EN()         (RCC->APB1ENR |= (1 << 30))  /*!< Enable clock for UART7  */
#define UART8_PCLK_EN()         (RCC->APB1ENR |= (1 << 31))  /*!< Enable clock for UART8  */



/**
 * @brief Clock Disable Macros for USART/UART on APB2 Bus
 *        Enable clock for USART peripherals on APB2 bus.
 */
#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))   /*!< Disable clock for USART1 */
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 5))   /*!< Disable clock for USART6 */

/**
 * @brief Clock Disable Macros for USART/UART on APB1 Bus
 *        Enable clock for USART/UART peripherals on APB1 bus.
 */
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))  /*!< Disable clock for USART2 */
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))  /*!< Disable clock for USART3 */
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 19))  /*!< Disable clock for UART4  */
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 20))  /*!< Disable clock for UART5  */
#define UART7_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 30))  /*!< Disable clock for UART7  */
#define UART8_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 31))  /*!< Disable clock for UART8  */

/*
 * @brief Clock Disable Macros for GPIOx Peripherals 
 * 
 * These macros disable the clock for the GPIO peripherals by clearing the 
 * respective bit in the AHB1ENR register. This effectively disables power 
 * to the specified GPIO port.
 */
#define GPIOA_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 0))  /*!< Disable clock for GPIOA */
#define GPIOB_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 1))  /*!< Disable clock for GPIOB */
#define GPIOC_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 2))  /*!< Disable clock for GPIOC */
#define GPIOD_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 3))  /*!< Disable clock for GPIOD */
#define GPIOE_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 4))  /*!< Disable clock for GPIOE */
#define GPIOF_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 5))  /*!< Disable clock for GPIOF */
#define GPIOG_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 6))  /*!< Disable clock for GPIOG */
#define GPIOH_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 7))  /*!< Disable clock for GPIOH */
#define GPIOI_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 8))  /*!< Disable clock for GPIOI */


/*
 * @brief Peripheral Reset Macros for GPIOx Peripherals 
 * 
 * These macros perform a reset of the GPIO peripherals by setting and 
 * clearing the respective bits in the AHB1RSTR (AHB1 peripheral reset register). 
 * The reset clears all settings and returns the peripheral to its default state.
 */
#define GPIOA_RESET()         do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)  /*!< Reset GPIOA */
#define GPIOB_RESET()         do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)  /*!< Reset GPIOB */
#define GPIOC_RESET()         do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)  /*!< Reset GPIOC */
#define GPIOD_RESET()         do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)  /*!< Reset GPIOD */
#define GPIOE_RESET()         do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)  /*!< Reset GPIOE */
#define GPIOF_RESET()         do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)  /*!< Reset GPIOF */
#define GPIOG_RESET()         do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)  /*!< Reset GPIOG */
#define GPIOH_RESET()         do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)  /*!< Reset GPIOH */
#define GPIOI_RESET()         do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)  /*!< Reset GPIOI */



//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


/************************************************************************************
 *                           End of Header File
 ************************************************************************************/

#endif 