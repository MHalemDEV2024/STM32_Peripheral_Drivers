/*
 * exti.h
 *
 * Created on: Sep 17, 2024
 * Author: Mohamed Saeed
 *
 * Description: This header file contains definitions and function prototypes for the EXTI and 
 *              SYSCFG peripherals for the STM32F429 microcontroller.
 */


#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f429xx.h"

/**
 * @brief Peripheral register definition structure for EXTI (External Interrupt/Event Controller)
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Interrupt Mask Register,                    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< Event Mask Register,                        Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< Rising Trigger Selection Register,          Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< Falling Trigger Selection Register,         Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< Software Interrupt Event Register,          Address offset: 0x10 */
	__vo uint32_t PR;     /*!< Pending Register,                           Address offset: 0x14 */
} EXTI_RegDef_t;

/**
 * @brief Peripheral register definition structure for SYSCFG (System Configuration Controller)
 */
typedef struct
{
	__vo uint32_t MEMRMP;        /*!< Memory Remap Register,                      Address offset: 0x00 */
	__vo uint32_t PMC;           /*!< Peripheral Mode Configuration,              Address offset: 0x04 */
	__vo uint32_t EXTICR[4];     /*!< External Interrupt Configuration Registers, Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< Reserved,                                   Address offset: 0x18-0x1C */
	__vo uint32_t CMPCR;         /*!< Compensation Cell Control Register,         Address offset: 0x20 */
	uint32_t      RESERVED2[2];  /*!< Reserved,                                   Address offset: 0x24-0x28 */
	__vo uint32_t CFGR;          /*!< Configuration Register,                     Address offset: 0x2C */
} SYSCFG_RegDef_t;

/***************************************************************************************************/
/*        Peripheral Definitions (Peripheral Base Addresses Typecasted to xxx_RegDef_t)            */
/***************************************************************************************************/

#define EXTI                  ((EXTI_RegDef_t*)EXTI_BASEADDR)     /*!< EXTI peripheral base address */
#define SYSCFG                ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR) /*!< SYSCFG peripheral base address */

/***************************************************************************************************/
/*                                Clock Enable Macros for SYSCFG Peripheral                        */
/***************************************************************************************************/

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14)) /*!< Enable clock for SYSCFG peripheral */

/***************************************************************************************************/
/*          IRQ (Interrupt Request) Numbers for STM32F429x MCU   (vector table, table63)           */
/***************************************************************************************************/

#define IRQ_NO_EXTI0          6  /*!< EXTI Line 0 Interrupt */
#define IRQ_NO_EXTI1          7  /*!< EXTI Line 1 Interrupt */
#define IRQ_NO_EXTI2          8  /*!< EXTI Line 2 Interrupt */
#define IRQ_NO_EXTI3          9  /*!< EXTI Line 3 Interrupt */
#define IRQ_NO_EXTI4         10  /*!< EXTI Line 4 Interrupt */
#define IRQ_NO_EXTI9_5       23  /*!< EXTI Lines [9:5] Interrupt */
#define IRQ_NO_EXTI15_10     40  /*!< EXTI Lines [15:10] Interrupt */


/***************************************************************************************************/
/*                                Macros for NVIC Interrupt Priority Levels                        */
/***************************************************************************************************/

#define NVIC_IRQ_PRI0        0   /*!< NVIC Priority Level 0 (Highest Priority) */
#define NVIC_IRQ_PRI15       15  /*!< NVIC Priority Level 15 (Lowest Priority) */


/************************************************************************************
 *                           End of Header File
 ************************************************************************************/


#endif /* INC_EXTI_H_ */
