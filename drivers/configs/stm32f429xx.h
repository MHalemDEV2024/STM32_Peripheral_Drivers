/******************************************************************************/ 
/*                           STM32F429xx Header File                          */
/*                                                                            */
/*  Created on: Sep 16, 2024                                                  */
/*  Author: Mohamed Saeed                                                     */
/*                                                                            */
/*  This file provides the base addresses and macros for the STM32F429xx      */
/*  microcontroller series. It includes peripheral base addresses, clock      */
/*  enable/disable macros, and reset macros for various peripherals.          */
/******************************************************************************/ 

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>

/* Macro to define volatile variables */
#define __vo volatile

/* Generic Macros for Boolean Values */
#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET
#define FLAG_RESET              RESET
#define FLAG_SET                SET

/******************************************************************************/ 
/*                  Base Addresses for Memory and Peripheral Buses            */ 
/******************************************************************************/ 

/**
 * @brief Flash and SRAM Base Addresses
 *        These addresses define the start of Flash memory, SRAM banks, and system ROM.
 */
#define FLASH_BASEADDR          0x08000000UL    /*!< Base address of Flash memory (up to 2 MB) */
#define SRAM1_BASEADDR          0x20000000UL    /*!< Base address of SRAM1 (112 KB) */
#define SRAM2_BASEADDR          0x2001C000UL    /*!< Base address of SRAM2 (16 KB) */
#define SRAM3_BASEADDR          0x20020000UL    /*!< Base address of SRAM3 (64 KB) */
#define ROM_BASEADDR            0x1FFF0000UL    /*!< Base address of System Memory (ROM, 30 KB) */
#define SRAM                    SRAM1_BASEADDR  /*!< Default alias for SRAM1 */

/**
 * @brief Peripheral Bus Base Addresses
 *        These addresses define the start of the APB1, APB2, AHB1, AHB2, and AHB3 peripheral buses.
 */
#define PERIPH_BASEADDR         0x40000000UL    /*!< Base address of the Peripheral Bus */

#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR /*!< Base address of APB1 (Advanced Peripheral Bus 1) */
#define APB2PERIPH_BASEADDR     0x40010000UL    /*!< Base address of APB2 (Advanced Peripheral Bus 2) */

#define AHB1PERIPH_BASEADDR     0x40020000UL    /*!< Base address of AHB1 (Advanced High-Performance Bus 1) */
#define AHB2PERIPH_BASEADDR     0x50000000UL    /*!< Base address of AHB2 (Advanced High-Performance Bus 2) */
#define AHB3PERIPH_BASEADDR     0xA0000000UL    /*!< Base address of AHB3 (Advanced High-Performance Bus 3) */

/******************************************************************************/ 
/*               AHB1 Peripheral Base Addresses (STM32F429)                   */ 
/*               Derived from STM32F429 Memory Map (RM0090, Table 1)           */ 
/******************************************************************************/ 

/**
 * @brief GPIO (General-Purpose Input/Output) Base Addresses
 *        Base addresses for GPIO peripheral blocks on the AHB1 bus. Each GPIO port
 *        is separated by an offset of 0x400 bytes.
 */
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)   /*!< Base address for GPIOA */
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)   /*!< Base address for GPIOB */
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)   /*!< Base address for GPIOC */
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)   /*!< Base address for GPIOD */
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)   /*!< Base address for GPIOE */
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)   /*!< Base address for GPIOF */
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)   /*!< Base address for GPIOG */
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)   /*!< Base address for GPIOH */
#define GPIOI_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2000)   /*!< Base address for GPIOI */
#define GPIOJ_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2400)   /*!< Base address for GPIOJ */
#define GPIOK_BASEADDR          (AHB1PERIPH_BASEADDR + 0x2800)   /*!< Base address for GPIOK */

/**
 * @brief RCC (Reset and Clock Control) Base Address
 *        The RCC registers control the system clock configuration, peripheral resets, and clock gating.
 */
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)   /*!< Base address for RCC */


/******************************************************************************/ 
/*               Include header files for peripheral drivers                  */ 
/******************************************************************************/ 

#include "gpio.h"
#include "rcc.h"


/************************************************************************************
 *                           End of Header File
 ************************************************************************************/

#endif /* INC_STM32F429XX_H_ */
