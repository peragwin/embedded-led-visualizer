/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
// TIM_HandleTypeDef htimx;
// TIM_HandleTypeDef htimy;
// DMA_HandleTypeDef hdma_timx;

uint16_t TimerPeriod = 100;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1080 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TimerPeriod;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = TimerPeriod - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

// /* TIM4 init function */
// void MX_TIMx_Init(void)
// {

//   TIM_ClockConfigTypeDef sClockSourceConfig;
//   TIM_MasterConfigTypeDef sMasterConfig;
//   TIM_OC_InitTypeDef sConfigOC;

//   htimx.Instance = TIMx;
//   htimx.Init.Prescaler = 0;
//   htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htimx.Init.Period = TIMx_PERIOD;
//   htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htimx.Init.RepetitionCounter = 0;
//   if (HAL_TIM_Base_Init(&htimx) != HAL_OK)
//     Error_Handler();

//   if (HAL_TIM_PWM_Init(&htimx) != HAL_OK)
//     Error_Handler();

//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig) != HAL_OK)
//     Error_Handler();

//   sMasterConfig.MasterOutputTrigger = TIMx_TRGO_OCREF;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig) != HAL_OK)
//     Error_Handler();

// /*
//   TIM_SlaveConfigTypeDef sSlaveConfig;
//   sSlaveConfig.TriggerPolarity = TIM_ETRPOLARITY_INVERTED;
//   sSlaveConfig.InputTrigger = TIM_TS_ITR3;
//   sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
//   if (HAL_TIM_SlaveConfigSynchronization(&htimy, &sSlaveConfig) != HAL_OK)
//     Error_Handler();
// */

//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = TIMx_PERIOD / 2;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//   if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIMx_CHANNEL) != HAL_OK)
//     Error_Handler();

//   //htimx.Instance->SMCR = 0x80B7;

//   HAL_TIM_MspPostInit(&htimx);
// }

// void MX_TIMy_Init(void)
// {
//   TIM_SlaveConfigTypeDef sSlaveConfig;
//   TIM_OC_InitTypeDef sConfigOC;

//   htimy.Instance  = TIMy;
//   htimy.Init.Period = 0xFFFFFFFF;
//   htimy.Init.Prescaler = 0;
//   htimy.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htimy.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htimy.Init.RepetitionCounter = 0;
//   if (HAL_TIM_Base_Init(&htimy) != HAL_OK)
//     Error_Handler();

//   if (HAL_TIM_OC_Init(&htimy) != HAL_OK) // tried "PWM"
//     Error_Handler();

//   sConfigOC.OCMode       = TIM_OCMODE_TIMING; //TIM_OCMODE_PWM2;
//   //TIM_OCMODE_TIMING
//   sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
//   sConfigOC.Pulse        = TIMy_COMPARE_VALUE;
//   sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
//   sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
//   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//   sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;

//   if (HAL_TIM_OC_ConfigChannel(&htimy, &sConfigOC, TIMy_CHANNEL) != HAL_OK) // tried "PWM"
//     Error_Handler();


//   /* Configure TIMy in External Clock mode1 slave mode &
//    use the Internal Trigger connected to TIMx TRGO as trigger source */
//   sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_EXTERNAL1;
//   sSlaveConfig.InputTrigger  = TIMy_TS;
//   if (HAL_TIM_SlaveConfigSynchronization(&htimy, &sSlaveConfig) != HAL_OK)
//     /* Configuration Error */
//     Error_Handler();

//   /* an attempt at chainging slave back to master. I don't think the timers support BOTH
//     master and slave mode at the same time 

//   TIM_MasterConfigTypeDef sMasterConfig;
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig) != HAL_OK)
//     Error_Handler();

//   htimy.Instance->SMCR = 0xb7;
//   htimy.Instance->CCMR1 = 0x78;
//   htimy.Instance->CR2 = 0x40;
//   */

//   HAL_TIM_MspPostInit(&htimy);
// }

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
   
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    /**TIM2 GPIO Configuration    
    PA0/WKUP     ------> TIM2_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }


  // else if(htim->Instance==TIMx)
  // {
  //   //HAL_GPIO_WritePin(TIMx_GPIO, TIMx_GPIO_PIN, GPIO_PIN_SET);
  //   GPIO_InitStruct.Pin = TIMx_GPIO_PIN;
  //   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //   GPIO_InitStruct.Pull = GPIO_NOPULL;
  //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //   GPIO_InitStruct.Alternate = TIMx_GPIO_AF;
  //   HAL_GPIO_Init(TIMx_GPIO, &GPIO_InitStruct);
  // }

/* used for debugging output when trying the master->slave->master config
  else if (htim->Instance==TIMy)
  {
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
*/

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

  // if(tim_baseHandle->Instance==TIMx)
  // {
    
  //   TIMx_CLK_ENABLE();
  
  //   hdma_timx.Instance = TIMx_DMA_STREAM;
  //   hdma_timx.Init.Channel = TIMx_DMA_CHANNEL;
  //   hdma_timx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  //   hdma_timx.Init.PeriphInc = DMA_PINC_DISABLE;
  //   hdma_timx.Init.MemInc = DMA_MINC_ENABLE;
  //   hdma_timx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  //   hdma_timx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  //   hdma_timx.Init.Mode = DMA_NORMAL;
  //   hdma_timx.Init.Priority = DMA_PRIORITY_HIGH;
  //   hdma_timx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  //   hdma_timx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  //   hdma_timx.Init.MemBurst = DMA_MBURST_INC16;
  //   hdma_timx.Init.PeriphBurst = DMA_PBURST_SINGLE;
  //   if (HAL_DMA_Init(&hdma_timx) != HAL_OK)
  //   {
  //     _Error_Handler(__FILE__, __LINE__);
  //   }

  //   __HAL_LINKDMA(tim_baseHandle,hdma[TIMx_DMA_ID], hdma_timx);

  //   /* Configure the NVIC for DMA                             */
  //   /* NVIC configuration for DMA transfer complete interrupt */
  //   /* lower priority than TIMy OC to make sure PWM is stopped as fast as possible */
  //   /* otherwise in double buffer mode, when end of frame is aligned with buffer size */
  //   /* the dma IT will occur at same time as TIMy OC IT and delay to stop PWM will will depend on frame size */
  //   HAL_NVIC_SetPriority(TIMx_DMA_IRQn, 1, 1);
  //   HAL_NVIC_EnableIRQ(TIMx_DMA_IRQn);
  // }

  // if(tim_baseHandle->Instance==TIMy)
  // {
  //   //TIMy_CLK_ENABLE();
  //   __HAL_RCC_TIM5_CLK_ENABLE();

  //   HAL_NVIC_SetPriority(TIMy_IRQn, 2, 1);
  //   HAL_NVIC_EnableIRQ(TIMy_IRQn);
  // }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }

  // if(tim_baseHandle->Instance==TIMx)
  // {
  // /* USER CODE BEGIN TIM4_MspDeInit 0 */

  // /* USER CODE END TIM4_MspDeInit 0 */
  //   /* Peripheral clock disable */
  //   __HAL_RCC_TIM8_CLK_DISABLE();

  //   /* TIM4 DMA DeInit */
  //   HAL_DMA_DeInit(tim_baseHandle->hdma[TIMx_DMA_ID]);
  // /* USER CODE BEGIN TIM4_MspDeInit 1 */

  // /* USER CODE END TIM4_MspDeInit 1 */
  // }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
