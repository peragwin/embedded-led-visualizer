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
#include "color.h"
#include "audio.h"
#include "display_grid.h"
#include <arm_math.h>

#define BUFFER_SIZE 76800
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))

static void CPU_CACHE_Enable(void);
void SystemClock_Config(void);

// void DMA2D_WriteBuffer(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t color);
// static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *hdma2d);
// static void DMA2D_TransferError(DMA2D_HandleTypeDef *hdma2d);

void LCD_SetBuffer(color_t *buffer, uint16_t x, uint16_t y, color_t c);
void LCD_FillRect_DMA2D(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t c);
void LCD_Buffer_DMA2D(void);
void LCD_TestDemo(void);
void LCD_ColorDemo(void);

static void display_grid_init(void);
static void render_display(void);

static void TransferComplete(DMA2D_HandleTypeDef *hdma2d);
static void TransferError(DMA2D_HandleTypeDef *hdma2d);

color_t buffer[BUFFER_SIZE] = {};
DisplayGrid_TypeDef *display_grid;

int bucket_enabled = 0;
int vsync_enabled = 1;
int wave_enabled = 0;
int display_grid_enabled = 1;

volatile int vsync_ready = 0;

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  CPU_CACHE_Enable();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_I2S1_Init();

  MX_DMA2D_Init();
  hdma2d.XferCpltCallback = TransferComplete;
  hdma2d.XferErrorCallback = TransferError;
  hdma2d_r2m.XferCpltCallback = TransferComplete;
  hdma2d_r2m.XferErrorCallback = TransferError;

  Audio_Init();
  display_grid_init();

  LCD_Init();

  uint16_t devcode = 0;
	devcode = LCD_ReadReg(0x0000);
	if (devcode == 0x8989) LED_Set(LED_BLUE, 1);

  LCD_Clear(RGB565(127, 0, 192));

  LCD_PutStr(10,10,"Hello fucking 16-bit interface!",RGB565(255,192,127));
	LCD_PutStr(30,30,"It's much faster and that's good!",RGB565(255,128,0));

  HAL_Delay(500);

  if (vsync_enabled) {
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    Error_Handler();
  }

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
  color_t green = {0, 0xfc, 0};
  color_t red = {0xff, 0, 0};
  color_t blue = {0,0, 0xff};
  color_t black = {0, 0, 0};
  float32_t scale = 1;

  LCD_FillRect_DMA2D(0, 0, 320, 240, green);
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
    // color_t c = {0,0,0};
    // for (int y = 32; y < 32+128; y++) {
    //   for (int x = 0; x < 320; x++) {
    //     LCD_SetBuffer(buffer, x, y, c);
    //   }
    // }

    Audio_ensure_i2s_frame_sync();

    if (display_grid_enabled) {
      render_display();
    }

    if (bucket_enabled) {
    float32_t *buckets = Audio_GetProcessedOutput();
    float32_t max;
    arm_max_f32(buckets, NUM_BUCKETS, &max, NULL);
    scale = .95 * scale + .05 * max;
    for (int i = 0; i < 32; i++) {
      int v = 100.0 * buckets[i] / scale;
      v = v > 120.0 ? 120.0 : v;
      v = 180 - v;
      for (int y = 60; y < 180; y++) {
        for (int x = 10*i; x < 10*(i+1); x++) {
          if (y < v) {
            LCD_SetBuffer(buffer, x, y, black);
          } else {
            LCD_SetBuffer(buffer, x, y, green);
          }
        }
      }
    }
    }

    if (wave_enabled) {
    int16_t *audio = Audio_GetBuffer(1);
    for (int x = 0; x < 320; x++) {
      int idx = 2*x;
      int y1 = 120 + (audio[idx] / 512);
      int y2 = 120 + (audio[idx+1] / 512);
      LCD_SetBuffer(buffer, x, y1, red);
      LCD_SetBuffer(buffer, x, y2, blue);
    }
    }
    
    if (vsync_enabled)
      while(!vsync_ready);
      vsync_ready = 0;
    LCD_Buffer_DMA2D();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    vsync_ready = 1;
  }
}

static void render_display(void) {
  Render(display_grid);
  int cols = display_grid->fs->columns;
  int rows = display_grid->fs->size;
  int xincr = 320 / cols;
  int yincr = 240 / rows;
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      color_t c = display_grid->display[i*rows + j];
      LCD_FillRect_DMA2D(i*xincr, ((rows-1-j)*yincr), xincr, yincr, c);
    }
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

void LCD_FillRect_DMA2D(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color_t c) {
  while(transfer_in_progress);
  transfer_in_progress = 1;

  uint16_t *loc = buffer + 320*y + x;
  uint32_t src = (c.r << 16) | (c.g << 8) | (c.b);
  DMA2D_FillRect(src, (uint32_t)loc, w, h); 
}

static void TransferComplete(DMA2D_HandleTypeDef *hdma2d) {
  static int toggle;

  transfer_in_progress = 0;
  toggle ^= 1;
  LED_Set(LED_GREEN, toggle);
}

static void TransferError(DMA2D_HandleTypeDef *hdma2d) {
  LED_Set(LED_RED, 1);
  Error_Handler();
}

static void display_grid_init(void) {
  display_grid = NewDisplayGrid(frequency_sensor);
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
