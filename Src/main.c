/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "fmc.h"
#include "tim.h"
#include "dma2d.h"
#include "i2s.h"
#include "dma.h"
#include "ssd1289.h"
#include "audio.h"
#include <arm_math.h>

#define BUFFER_SIZE 76800
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))

static void CPU_CACHE_Enable(void);
void SystemClock_Config(void);

// void DMA2D_WriteBuffer(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t color);
// static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *hdma2d);
// static void DMA2D_TransferError(DMA2D_HandleTypeDef *hdma2d);

void LCD_SetBuffer(color_t *buffer, uint16_t x, uint16_t y, color_t c);
void LCD_Buffer_DMA2D(void);
void LCD_TestDemo(void);
void LCD_ColorDemo(void);

static void TransferComplete(DMA2D_HandleTypeDef *hdma2d);
static void TransferError(DMA2D_HandleTypeDef *hdma2d);

color_t buffer[BUFFER_SIZE] = {};

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  //CPU_CACHE_Enable();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_TIM2_Init();
  MX_I2S1_Init();

  MX_DMA2D_Init();
  hdma2d.XferCpltCallback = TransferComplete;
  hdma2d.XferErrorCallback = TransferError;

  Audio_Init();

  LCD_Init();

  uint16_t devcode = 0;
	devcode = LCD_ReadReg(0x0000);
	if (devcode == 0x8989) LED_Set(LED_BLUE, 1);

  LCD_Clear(RGB565(127, 0, 192));

  LCD_PutStr(10,10,"Hello fucking 16-bit interface!",RGB565(255,192,127));
	LCD_PutStr(30,30,"It's much faster and that's good!",RGB565(255,128,0));

  HAL_Delay(2000);

  LCD_ColorDemo();
  //LCD_TestDemo();

}

void LCD_TestDemo(void) {
  int j = 0;
  while (1) {
    j++;

    uint8_t r,g,b;
    r = (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/120.0 * (j)), 255);
    b = (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/120.0 * (j) + 2*PI/3), 255);
    g = (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/120.0 * (j) - 2*PI/3), 255);
    LCD_Clear(RGB565(r, g, b));

  //   LCD_PutStr(10,10,"Hello fucking 16-bit interface!",RGB565(255,192,127));
  //   LCD_PutStr(30,30,"It's much faster and that's good!",RGB565(255,128,0));
  // //uint8_t v = toggle ? 0x55 : 0xaa;
  //   LCD_FillRect(100, 100, 50, 100, RGB565(0x55, 0x55, 0x55));
    
    // LCD_write_data(toggle ? 0xffff : 0);
    HAL_Delay(20);
  }
}

#define ABS(a) (a < 0 ? -a : a)

color_t hsv(float h, float s, float v) {
  h = fmod(h, 360);
  h /= 60;
  float c = s * v;
  float x = c * (1-ABS(fmod(h, 2.0)-1));
  float m = v - c;
  float r, g, b;
  if (h < 1.0) {
    r = c; g = x;
  } else if (h < 2.0) {
    r = x; g = c;
  } else if (h < 3.0) {
    g = c; b = x;
  } else if (h < 4.0) {
    g = x; b = c;
  } else if (h < 5.0) {
    r = x; b = c;
  } else {
    r = c; b = x;
  }
  color_t color = {250*(m+r), 250*(m+g), 250*(m+b)};
  return color;
}

color_t fixBits(color_t c) {
  c.r &= 0xfc;
  c.g &= 0xfc;
  c.b &= 0xfc;
  return c;
}

color_t sim565(color_t c) {
  c.r &= 0xf8;
  c.g &= 0xfc;
  c.b &= 0xf8;
  return c;
}

void LCD_ColorDemo(void) {
  float r, g, b = 0;
  float incr = 1.44;
  for (int y = 0; y < 240; y++){
    for (int x = 0; x < 320; x++) {
      r += incr;
      if (r > 63) {
        r = 0;
        b += incr;
      }
      if (b > 63) {
        b = 0;
        g += incr;
      }
      color_t c = {(uint8_t)r<<2, 0&(uint8_t)g<<2, (uint8_t)b<<2};
      //int v = (x + y) % 2;
      //color_t c = {0xff*v, 0xff*v, 0xff*v};
      LCD_SetBuffer(buffer, x, y, c);
    }
  }

  int j = 0;
  color_t c;
  while (1) {
    j++;

    // for (int y = 0; y < 240; y++){
    //   // float h = 360.0 * (float)(4*j+y) / 480.0;
    //   // // c.r = 0xfc & (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/480.0 * (4*j+y)), 255);
    //   // // c.b = 0xfc & (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/480.0 * (4*j+y) + 2*PI/3), 255);
    //   // // c.g = 0xfc & (uint8_t) MIN(128 + 128*arm_sin_f32(2.0*PI/480.0 * (4*j+y) - 2*PI/3), 255);
    //   // c = fixBits(hsv(h, 1, 1));
    //   // for (int x = 0; x < 160; x++) {
    //   //   //float v = (160.0 - (float)x)/160.0;
    //   //   LCD_SetBuffer(buffer, x, y, c);
    //   // }
    //   // // c.r &= 0xf8;
    //   // // c.b &= 0xf8;
    //   // c = sim565(hsv(h, 1, 1));
    //   for (int x = 0; x < 320; x++) {
    //     // float v = ((float)x - 160.0)/160.0;
    //     c.r = (240*x+y+100*j)/130 % 255;
    //     c.b = (240*x+y+100*j)/190 % 255;
    //     c.g = (240*x*y+100*j)/230 % 255;
    //     LCD_SetBuffer(buffer, x, y, fixBits(c));
    //   }
    // }
    color_t c = {0,0,0};
    for (int y = 32; y < 32+128; y++) {
      for (int x = 0; x < 320; x++) {
        LCD_SetBuffer(buffer, x, y, c);
      }
    }

    if (hi2s1.Instance->SR & 0x100) {
      __HAL_I2S_DISABLE(&hi2s1);
      LED_Set(LED_RED, 1);

      HAL_Delay(1);

      __HAL_I2S_ENABLE(&hi2s1);      
      LED_Set(LED_RED, 0);
    }

    c.g = 0xfc;
    color_t c2 = {255, 0, 0};
    int16_t *audio = Audio_GetBuffer(0);
    for (int x = 0; x < 320; x++) {
      int idx = 2*x;
      int y1 = 96 + (audio[idx] / 512);
      int y2 = 96 + (audio[idx+1] / 512);
      LCD_SetBuffer(buffer, x, y1, c);
      LCD_SetBuffer(buffer, x, y2, c2);
    }
    
    LCD_Buffer_DMA2D();
  }
}

void LCD_SetBuffer(color_t *buffer, uint16_t x, uint16_t y, color_t c) {
  buffer[320*y+x] = c;
}

__IO int transfer_in_progress;

void LCD_Buffer_DMA2D(void) {
  while(transfer_in_progress) {
    HAL_Delay(1);
  }
  transfer_in_progress = 1;

  //LCD_Clear(0xaaaa);

  LCD_SetWindow(0, 0, 320, 240);
  LCD_write_command(0x0022);

  DMA2D_WriteBuffer((uint32_t) buffer, (uint32_t) &(LCD_DATA->REG), BUFFER_SIZE);
}

static void TransferComplete(DMA2D_HandleTypeDef *hdma2d) {
  static int toggle;

  transfer_in_progress = 0;
  toggle ^= 1;
  LED_Set(LED_GREEN, toggle);
}

static void TransferError(DMA2D_HandleTypeDef *hdma2d) {
  LED_Set(LED_RED, 1);
}

static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 2);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  LED_Set(LED_RED, 1);
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
