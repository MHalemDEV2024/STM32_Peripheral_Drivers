/*
 * gpio.h
 *
 * Created on: Sep 16, 2024
 * Author: Mohamed Saeed
 *
 * Description: This header file contains definitions and function prototypes for the GPIO driver
 *               for the STM32F429 microcontroller.
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stdarg.h>

#include "stm32f429xx.h"


/************************************************************************************
 *                           Data Structures
 ************************************************************************************/

/* GPIO Register Definition Structure */
typedef struct
{
    __vo uint32_t MODER;      /*!< GPIO port mode register,                             Address offset: 0x00 */
    __vo uint32_t OTYPER;     /*!< GPIO port output type register,                      Address offset: 0x04 */
    __vo uint32_t OSPEEDR;    /*!< GPIO port output speed register,                     Address offset: 0x08 */
    __vo uint32_t PUPDR;      /*!< GPIO port pull-up/pull-down register,                Address offset: 0x0C */
    __vo uint32_t IDR;        /*!< GPIO port input data register,                       Address offset: 0x10 */
    __vo uint32_t ODR;        /*!< GPIO port output data register,                      Address offset: 0x14 */
    __vo uint32_t BSRR;       /*!< GPIO port bit set/reset register,                    Address offset: 0x18 */
    __vo uint32_t LCKR;       /*!< GPIO port configuration lock register,               Address offset: 0x1C */
    __vo uint32_t AFR[2];     /*!< AFR[0]: GPIO alternate function low register,
                                   AFR[1]: GPIO alternate function high register,       Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;        /*!< GPIO pin number */
    uint8_t GPIO_PinMode;           /*!< GPIO pin mode, possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;          /*!< GPIO pin speed, possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;    /*!< GPIO pin pull-up/pull-down control */
    uint8_t GPIO_PinOPType;         /*!< GPIO pin output type */
    uint8_t GPIO_PinAltFunMode;     /*!< GPIO pin alternate function mode */
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;          /*!< Base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig; /*!< GPIO pin configuration settings */
} GPIO_Handle_t;




/***************************************************************************************************/
/*        Peripheral Definitions (Peripheral Base Addresses Typecasted to GPIO_RegDef_t)           */
/***************************************************************************************************/

#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)   /*!< Pointer to GPIOA registers */
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)   /*!< Pointer to GPIOB registers */
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)   /*!< Pointer to GPIOC registers */
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)   /*!< Pointer to GPIOD registers */
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)   /*!< Pointer to GPIOE registers */
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)   /*!< Pointer to GPIOF registers */
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)   /*!< Pointer to GPIOG registers */
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)   /*!< Pointer to GPIOH registers */
#define GPIOI               ((GPIO_RegDef_t*)GPIOI_BASEADDR)   /*!< Pointer to GPIOI registers */
#define GPIOJ               ((GPIO_RegDef_t*)GPIOJ_BASEADDR)   /*!< Pointer to GPIOI registers */
#define GPIOK               ((GPIO_RegDef_t*)GPIOK_BASEADDR)   /*!< Pointer to GPIOI registers */



/************************************************************************************
 *                           Macro Definitions
 ************************************************************************************/

/*
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/*
 * GPIO pin modes
 */
#define GPIO_MODE_IN        0   /*!< Input mode */
#define GPIO_MODE_OUT       1   /*!< Output mode */
#define GPIO_MODE_ALTFN     2   /*!< Alternate function mode */
#define GPIO_MODE_ANALOG    3   /*!< Analog mode */

//Interrupt mode
#define GPIO_MODE_IT_FT     4   /*!< Interrupt mode with falling edge trigger */
#define GPIO_MODE_IT_RT     5   /*!< Interrupt mode with rising edge trigger */
#define GPIO_MODE_IT_RFT    6   /*!< Interrupt mode with rising and falling edge trigger */

/*
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP     0   /*!< Push-pull output type */
#define GPIO_OP_TYPE_OD     1   /*!< Open-drain output type */

/*
 * GPIO pin speeds
 */
#define GPIO_SPEED_LOW      0   /*!< Low speed */
#define GPIO_SPEED_MEDIUM   1   /*!< Medium speed */
#define GPIO_SPEED_FAST     2   /*!< Fast speed */
#define GPIO_SPEED_HIGH     3   /*!< High speed */

/*
 * GPIO pin pull-up and pull-down configuration
 */
#define GPIO_NO_PUPD        0   /*!< No pull-up, no pull-down */
#define GPIO_PIN_PU         1   /*!< Pull-up */
#define GPIO_PIN_PD         2   /*!< Pull-down */


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)



/************************************************************************************
 *                           Function Prototypes
 ************************************************************************************/


/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_PinInit(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t PinMode, uint8_t PinSpeed, uint8_t PinOPType, uint8_t PinPuPdControl,uint8_t PinAltFunMode);
void GPIO_PinInitV(GPIO_RegDef_t *pGPIOx, uint8_t numPins, ...);
void GPIO_PortInit(GPIO_RegDef_t *pGPIOx);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_TogglePort(GPIO_RegDef_t *pGPIOx);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/************************************************************************************
 *                           End of Header File
 ************************************************************************************/

#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
