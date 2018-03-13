/**
  ******************************************************************************
  * File Name          : TIM.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
// extern TIM_HandleTypeDef htimx;
// extern TIM_HandleTypeDef htimy;
// DMA_HandleTypeDef hdma_timx;

/* USER CODE BEGIN Private defines */

// TIMx is used to drive the memory->gpio DMA
// It is configured in PWM, and the output is on PC_6
#define TIMx TIM8
// #define TIMx TIM12
#define TIMx_CHANNEL TIM_CHANNEL_1
#define TIMx_PERIOD 12 // ~10MHz
#define TIMx_TRGO_OCREF TIM_TRGO_OC1REF
#define TIMx_DMA_ID TIM_DMA_ID_CC1
#define TIMx_DMA_CC TIM_DMA_CC1

// #define TIMx_GPIO GPIOB
// #define TIMx_GPIO_PIN GPIO_PIN_14
// #define TIMx_GPIO_AF GPIO_AF9_TIM12
#define TIMx_GPIO GPIOC
#define TIMx_GPIO_PIN GPIO_PIN_6
#define TIMx_GPIO_AF GPIO_AF3_TIM8

#define TIMx_CLK_ENABLE __HAL_RCC_TIM8_CLK_ENABLE
// #define TIMx_CLK_ENABLE __HAL_RCC_TIM12_CLK_ENABLE

#define TIMx_DMA_CHANNEL DMA_CHANNEL_7
#define TIMx_DMA_STREAM  DMA2_Stream2
#define TIMx_DMA_IRQn DMA2_Stream2_IRQn
#define TIMx_DMA_IRQHandler DMA2_Stream2_IRQHandler


// TIMy is used to count the total number of DMA transfers since the overall
// number is greater than 0xFFFF (the maximum for the internal DMA counter).
// TIM5 is a 32bit counter
#define TIMy TIM5
#define TIMy_CHANNEL TIM_CHANNEL_1
#define TIMy_TS TIM_CLOCKSOURCE_ITR3 // wires this slave to TIM8
#define TIMy_COMPARE_VALUE 320*240 // the full frame

#define TIMy_CLK_ENABLE __HAL_RCC_TIM5_CLK_ENABLE

#define TIMy_IRQn TIM5_IRQn
#define TIMy_IRQHandler TIM5_IRQHandler


/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_TIM2_Init(void);
// void MX_TIMx_Init(void);
// void MX_TIMy_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
