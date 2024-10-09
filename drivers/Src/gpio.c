/*
 * gpio.c
 *
 *  Created on: Sep 16, 2024 
 *      Author: Mohamed Saeed
 *
 *  Description: This source file contains the implementation of the GPIO driver functions
 *               for the STM32F429 microcontroller.
 */
#include <string.h>

#include "stm32f429xx.h"

/************************************************************************************
 *                           Peripheral Clock Control
 ************************************************************************************/

/**
 * @fn      		  - GPIO_PeriClockControl
 * @brief             - Enables or disables the peripheral clock for the given GPIO port.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @param[in]         - EnorDi: ENABLE or DISABLE macros
 * @return            - None
 * @Note              - None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		switch ((uint32_t)pGPIOx)
		{
			case (uint32_t)GPIOA: GPIOA_CLK_EN(); break;
			case (uint32_t)GPIOB: GPIOB_CLK_EN(); break;
			case (uint32_t)GPIOC: GPIOC_CLK_EN(); break;
			case (uint32_t)GPIOD: GPIOD_CLK_EN(); break;
			case (uint32_t)GPIOE: GPIOE_CLK_EN(); break;
			case (uint32_t)GPIOF: GPIOF_CLK_EN(); break;
			case (uint32_t)GPIOG: GPIOG_CLK_EN(); break;
			case (uint32_t)GPIOH: GPIOH_CLK_EN(); break;
			case (uint32_t)GPIOI: GPIOI_CLK_EN(); break;
		}
	}
	else
	{
		switch ((uint32_t)pGPIOx)
		{
			case (uint32_t)GPIOA: GPIOA_CLK_DI(); break;
			case (uint32_t)GPIOB: GPIOB_CLK_DI(); break;
			case (uint32_t)GPIOC: GPIOC_CLK_DI(); break;
			case (uint32_t)GPIOD: GPIOD_CLK_DI(); break;
			case (uint32_t)GPIOE: GPIOE_CLK_DI(); break;
			case (uint32_t)GPIOF: GPIOF_CLK_DI(); break;
			case (uint32_t)GPIOG: GPIOG_CLK_DI(); break;
			case (uint32_t)GPIOH: GPIOH_CLK_DI(); break;
			case (uint32_t)GPIOI: GPIOI_CLK_DI(); break;
		}
	}
}

/************************************************************************************
 *                           GPIO Initialization and De-initialization
 ************************************************************************************/

/**
 * @fn      		  - GPIO_Init
 * @brief             - Initializes the GPIO pin according to the specified parameters.
 * @param[in]         - pGPIOHandle: Pointer to GPIO handle structure
 * @return            - None
 * @Note              - None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
		pGPIOHandle->pGPIOx->MODER |= temp; // Set
	}else
	{
		// Interrupt mode configuration can be added later

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
		    //1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
			{
					//1 . configure the RTSR
					EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					//Clear the corresponding RTSR bit
					EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
			{
				//1. configure both FTSR and RTSR
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding RTSR bit
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}

			//2. configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

			//3 . enable the exti interrupt delivery using IMR
			EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Set

	// 3. Configure the pull-up/pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
	pGPIOHandle->pGPIOx->PUPDR |= temp; // Set

	// 4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear
	pGPIOHandle->pGPIOx->OTYPER |= temp; // Set

	// 5. Configure the alternate function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Set
	}

}

// Function to initialize an entire GPIO port
void GPIO_PortInit(GPIO_RegDef_t *pGPIOx) {
    GPIO_Handle_t GpioPort;

    for (uint8_t pinNumber = 0; pinNumber < 16; pinNumber++) {
        GpioPort.pGPIOx = pGPIOx;
        GpioPort.GPIO_PinConfig.GPIO_PinNumber = pinNumber;
        GpioPort.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
        GpioPort.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
        GpioPort.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
        GpioPort.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

        GPIO_Init(&GpioPort);
    }
}
// Function to initialize a single GPIO pin
void GPIO_PinInit(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t PinMode, uint8_t PinSpeed, uint8_t PinOPType, uint8_t PinPuPdControl,uint8_t PinAltFunMode) {
    GPIO_Handle_t GpioPin;
    memset(&GpioPin,0,sizeof(GpioPin));

    GpioPin.pGPIOx = pGPIOx;
    GpioPin.GPIO_PinConfig.GPIO_PinNumber = PinNumber;
    GpioPin.GPIO_PinConfig.GPIO_PinMode = PinMode;
    GpioPin.GPIO_PinConfig.GPIO_PinSpeed = PinSpeed;
    GpioPin.GPIO_PinConfig.GPIO_PinOPType = PinOPType;
    GpioPin.GPIO_PinConfig.GPIO_PinPuPdControl = PinPuPdControl;
		GpioPin.GPIO_PinConfig.GPIO_PinAltFunMode = PinAltFunMode;

    GPIO_Init(&GpioPin);
}

// Function to initialize multiple GPIO pins with various configurations
void GPIO_PinInitV(GPIO_RegDef_t *pGPIOx, uint8_t numPins, ...) {
    va_list args;
    va_start(args, numPins);

    for (uint8_t i = 0; i < numPins; i++) {
        GPIO_Handle_t GpioPin;

        // Fetch the GPIO pin number and configuration details
        GpioPin.pGPIOx = pGPIOx;
        GpioPin.GPIO_PinConfig.GPIO_PinNumber = va_arg(args, int);
        GpioPin.GPIO_PinConfig.GPIO_PinMode = va_arg(args, int);
        GpioPin.GPIO_PinConfig.GPIO_PinSpeed = va_arg(args, int);
        GpioPin.GPIO_PinConfig.GPIO_PinOPType = va_arg(args, int);
        GpioPin.GPIO_PinConfig.GPIO_PinPuPdControl = va_arg(args, int);

        GPIO_Init(&GpioPin);
    }

    va_end(args);
}

/**
 * @fn      		  - GPIO_DeInit
 * @brief             - De-initializes the GPIO port, resetting all registers.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @return            - None
 * @Note              - None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	switch ((uint32_t)pGPIOx)
	{
		case (uint32_t)GPIOA: GPIOA_RESET(); break;
		case (uint32_t)GPIOB: GPIOB_RESET(); break;
		case (uint32_t)GPIOC: GPIOC_RESET(); break;
		case (uint32_t)GPIOD: GPIOD_RESET(); break;
		case (uint32_t)GPIOE: GPIOE_RESET(); break;
		case (uint32_t)GPIOF: GPIOF_RESET(); break;
		case (uint32_t)GPIOG: GPIOG_RESET(); break;
		case (uint32_t)GPIOH: GPIOH_RESET(); break;
		case (uint32_t)GPIOI: GPIOI_RESET(); break;
	}
}

/************************************************************************************
 *                           GPIO Read and Write Operations
 ************************************************************************************/

/**
 * @fn      		  - GPIO_ReadFromInputPin
 * @brief             - Reads the value from a specific GPIO pin.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @param[in]         - PinNumber: Pin number to read
 * @return            - Pin state (0 or 1)
 * @Note              - None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
}

/**
 * @fn      		  - GPIO_ReadFromInputPort
 * @brief             - Reads the value from an entire GPIO port.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @return            - Port value
 * @Note              - None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/**
 * @fn      		  - GPIO_WriteToOutputPin
 * @brief             - Writes a value to a specific GPIO pin.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @param[in]         - PinNumber: Pin number to write to
 * @param[in]         - Value: Value to write (0 or 1)
 * @return            - None
 * @Note              - None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**
 * @fn      		  - GPIO_WriteToOutputPort
 * @brief             - Writes a value to an entire GPIO port.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @param[in]         - Value: Value to write to the port
 * @return            - None
 * @Note              - None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**
 * @fn      		  - GPIO_ToggleOutputPin
 * @brief             - Toggles the state of a specific GPIO pin.
 * @param[in]         - pGPIOx: Base address of the GPIO peripheral
 * @param[in]         - PinNumber: Pin number to toggle
 * @return            - None
 * @Note              - None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

// Function to toggle all pins on a GPIO port
void GPIO_TogglePort(GPIO_RegDef_t *pGPIOx) {
    pGPIOx->ODR ^= 0xFFFF; // Toggle all 16 pins on the port
}



/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - Configures the interrupt in the NVIC (Enable or Disable)
 *
 * @param[in]         - IRQNumber: Interrupt Request number
 * @param[in]         - EnorDi: ENABLE or DISABLE
 *
 * @return            - None
 *
 * @Note              - None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Configures the priority of the interrupt
 *
 * @param[in]         - IRQNumber: Interrupt Request number
 * @param[in]         - IRQPriority: Priority level (0 to 15)
 *
 * @return            - None
 *
 * @Note              - None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) &= ~(0xFF << (8 * iprx_section));  // Clear the bits first
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);   // Set the priority
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Clears the EXTI PR register bit for the given pin number
 *
 * @param[in]         - PinNumber: GPIO pin number (0 to 15)
 *
 * @return            - None
 *
 * @Note              - None
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);  // Clear the pending bit
	}
}


