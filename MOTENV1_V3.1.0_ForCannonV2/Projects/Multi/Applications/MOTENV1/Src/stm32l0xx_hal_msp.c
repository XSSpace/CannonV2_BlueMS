/**
  ******************************************************************************
  * @file    stm32l0xx_hal_msp.c
  * @author  Central Lab
  * @version V3.1.0
  * @date    12-July-2017
  * @brief   HAL MSP module.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by STM32CubeMX and eventually modified 
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "main.h"
#include "stm32l0xx_periph_conf.h"

/** @addtogroup STM32L0xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/**
 * @brief  I2C MSP Initialization
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable I2C GPIO clocks */
  I2C1_CMN_DEFAULT_SCL_SDA_GPIO_CLK_ENABLE();
  
  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin = I2C1_CMN_DEFAULT_SCL_PIN |I2C1_CMN_DEFAULT_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  =I2C1_CMN_DEFAULT_SCL_SDA_AF;
  HAL_GPIO_Init(I2C1_CMN_DEFAULT_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  
  /* Enable the I2C_EXPBD peripheral clock */
 I2C1_CMN_DEFAULT_CLK_ENABLE();
  
  /* Force the I2C peripheral clock reset */
 I2C1_CMN_DEFAULT_FORCE_RESET();
  
  /* Release the I2C peripheral clock reset */
 I2C1_CMN_DEFAULT_RELEASE_RESET();
  
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C1_CMN_DEFAULT_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_CMN_DEFAULT_EV_IRQn);
}

/**
 * @brief SPI MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration  
 * @param  hspi: SPI handle.
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Enable peripherals clock */

  /* Enable GPIO Ports Clock */  
  SPI1_CMN_DEFAULT_RESET_CLK_ENABLE();
  SPI1_CMN_DEFAULT_SCLK_CLK_ENABLE();
  SPI1_CMN_DEFAULT_MISO_CLK_ENABLE();
  SPI1_CMN_DEFAULT_MOSI_CLK_ENABLE();
  SPI1_CMN_DEFAULT_CS_CLK_ENABLE();
  SPI1_CMN_DEFAULT_IRQ_CLK_ENABLE();

  /* Enable SPI clock */
  SPI1_CMN_DEFAULT_CLK_ENABLE();

  /* Reset */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_RESET_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_RESET_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_RESET_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_RESET_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_RESET_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_RESET_PORT, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(SPI1_CMN_DEFAULT_RESET_PORT, SPI1_CMN_DEFAULT_RESET_PIN, GPIO_PIN_RESET);	/*Added to avoid spurious interrupt from the BlueNRG */

  /* SCLK */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_SCLK_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_SCLK_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_SCLK_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_SCLK_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_SCLK_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_SCLK_PORT, &GPIO_InitStruct); 

  /* MISO */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_MISO_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_MISO_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_MISO_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_MISO_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_MISO_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_MISO_PORT, &GPIO_InitStruct);

  /* MOSI */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_MOSI_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_MOSI_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_MOSI_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_MOSI_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_MOSI_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_MOSI_PORT, &GPIO_InitStruct);

  /* NSS/CSN/CS */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_CS_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_CS_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_CS_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_CS_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_CS_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SPI1_CMN_DEFAULT_CS_PORT, SPI1_CMN_DEFAULT_CS_PIN, GPIO_PIN_SET);

  /* IRQ -- INPUT */
  GPIO_InitStruct.Pin = SPI1_CMN_DEFAULT_IRQ_PIN;
  GPIO_InitStruct.Mode = SPI1_CMN_DEFAULT_IRQ_MODE;
  GPIO_InitStruct.Pull = SPI1_CMN_DEFAULT_IRQ_PULL;
  GPIO_InitStruct.Speed = SPI1_CMN_DEFAULT_IRQ_SPEED;
  GPIO_InitStruct.Alternate = SPI1_CMN_DEFAULT_IRQ_ALTERNATE;
  HAL_GPIO_Init(SPI1_CMN_DEFAULT_IRQ_PORT, &GPIO_InitStruct);

  /* Configure the NVIC for SPI */  
  HAL_NVIC_SetPriority(SPI1_CMN_DEFAULT_EXTI_IRQn, 3, 0);    
  HAL_NVIC_EnableIRQ(SPI1_CMN_DEFAULT_EXTI_IRQn);
}

/**
  * @brief TIM OC MSP Initialization
  * This function configures the hardware resources used in this example:
  *  - Peripheral's clock enable
  *  - Peripheral's Interrupt Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{ 
  /* TIM1 Peripheral clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /*  Enable the TIM1 global Interrupt & set priority */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0x8, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
