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
/*                  ARM Cortex Mx Processor-Specific Details                  */
/*         Definitions for NVIC (Nested Vector Interrupt Controller)          */
/*                 register addresses and priority settings.                  */
/*           (cortex_M4 Peripherla, Table 4-2 NVIC register summary)          */ 
/******************************************************************************/ 

/*
 * ARM Cortex Mx Processor NVIC ISERx (Interrupt Set-Enable Registers) Addresses
 * 
 * The ISER registers enable interrupts for the corresponding interrupt numbers.
 * Each bit in these registers corresponds to a particular interrupt line.
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 ) /*!< Base address for ISER0 register */
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 ) /*!< Base address for ISER1 register */
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 ) /*!< Base address for ISER2 register */
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C ) /*!< Base address for ISER3 register */


/*
 * ARM Cortex Mx Processor NVIC ICERx (Interrupt Clear-Enable Registers) Addresses
 * 
 * The ICER registers clear interrupts for the corresponding interrupt numbers.
 * Each bit in these registers corresponds to a particular interrupt line.
 */

#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180) /*!< Base address for ICER0 register */
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184) /*!< Base address for ICER1 register */
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188) /*!< Base address for ICER2 register */
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C) /*!< Base address for ICER3 register */


/*
 * ARM Cortex Mx Processor NVIC PR (Priority Register) Base Address
 * 
 * The NVIC PR registers are used to set the priority of each interrupt.
 * Each interrupt has its own priority value, which determines its preemption level.
 */

#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400) /*!< Base address for NVIC priority registers */


/*
 * ARM Cortex Mx Processor Number of Priority Bits Implemented
 * 
 * The Cortex Mx processors typically implement a subset of the available priority levels.
 * This macro defines the number of priority bits implemented by the processor.
 */

#define NO_PR_BITS_IMPLEMENTED  4  /*!< Number of priority bits implemented in NVIC */


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
/*               APB2 Peripheral Base Addresses (STM32F429)                   */ 
/*               Derived from STM32F429 Memory Map (RM0090, Table 1)           */ 
/******************************************************************************/ 


/**
 * @brief EXTI (External Interrupt/Event Controller) Base Address
 *        The EXTI handles external interrupt/event requests from peripherals and external pins.
 */
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)   /*!< Base address for EXTI */

/**
 * @brief SYSCFG (System Configuration Controller) Base Address
 *        The SYSCFG provides system configuration functions, including external interrupt configuration.
 */
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)   /*!< Base address for SYSCFG */


/******************************************************************************/ 
/*               Include header files for peripheral drivers                  */ 
/******************************************************************************/ 

#include "gpio.h"
#include "rcc.h"
#include "exti.h"


/************************************************************************************
 *                           End of Header File
 ************************************************************************************/

#endif /* INC_STM32F429XX_H_ */
