#include "stm32f429xx.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1 = 0;  // Initialize pclk1
    uint32_t SystemClk = 0;  // Initialize SystemClk
    uint8_t clksrc, temp, ahbp, apb1p;

    // Get the clock source
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    // Determine the system clock based on the clock source
    if (clksrc == 0)
    {
        SystemClk = 16000000;  // HSI selected
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;   // HSE selected
    }
    else if (clksrc == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();  // PLL selected
    }

    // Calculate AHB prescaler
    temp = ((RCC->CFGR >> 4) & 0xF);
    if (temp < 8)
    {
        ahbp = 1;  // No division
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // Calculate APB1 prescaler
    temp = ((RCC->CFGR >> 10) & 0x7);
    if (temp < 4)
    {
        apb1p = 1;  // No division
    }
    else
    {
        apb1p = APB1_PreScaler[temp - 4];
    }

    // Calculate PCLK1
    pclk1 = (SystemClk / ahbp) / apb1p;

    return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - Calculate the clock value for PCLK2
 *
 * @return            - Clock value for PCLK2
 *
 *********************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t pclk2 = 0;  // Initialize pclk2
    uint32_t SystemClock = 0;  // Initialize SystemClock
    uint8_t clk_src = (RCC->CFGR >> 2) & 0x3;
    uint8_t ahbp, apb2p;

    // Determine the system clock based on the clock source
    if (clk_src == 0)
    {
        SystemClock = 16000000;  // HSI selected
    }
    else if (clk_src == 1)
    {
        SystemClock = 8000000;   // HSE selected
    }
    else if (clk_src == 2)
    {
        SystemClock = RCC_GetPLLOutputClock();  // PLL selected
    }

    // Calculate AHB prescaler
    uint8_t tmp = (RCC->CFGR >> 4) & 0xF;
    if (tmp < 0x08)
    {
        ahbp = 1;  // No division
    }
    else
    {
        ahbp = AHB_PreScaler[tmp - 8];
    }

    // Calculate APB2 prescaler
    tmp = (RCC->CFGR >> 13) & 0x7;
    if (tmp < 0x04)
    {
        apb2p = 1;  // No division
    }
    else
    {
        apb2p = APB1_PreScaler[tmp - 4];
    }

    // Calculate PCLK2
    pclk2 = (SystemClock / ahbp) / apb2p;

    return pclk2;
}

uint32_t RCC_GetPLLOutputClock()
{
    // Implementation for PLL output clock calculation should go here
    // For now, return 0 or a placeholder value
    return 0;
}
