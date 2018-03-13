#include "dma2d.h"
#include "gpio.h"

DMA2D_HandleTypeDef hdma2d;
DMA2D_HandleTypeDef hdma2d_r2m;

static void DMA2D_M2M_Init(void);
static void DMA2D_R2M_Init(void);

volatile int dma_mode;

/* DMA2D init function */
void MX_DMA2D_Init(void)
{
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
  hdma2d.Init.OutputOffset = 0;

  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_SWAP;

  hdma2d_r2m.Instance = DMA2D;
  hdma2d_r2m.Init.Mode = DMA2D_R2M;
  hdma2d_r2m.Init.ColorMode = DMA2D_RGB888;
  hdma2d_r2m.Init.OutputOffset = 0;

  DMA2D_R2M_Init();
}

static void DMA2D_M2M_Init(void)
{
  if (dma_mode != DMA2D_M2M) {
    dma_mode = DMA2D_M2M;
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
      _Error_Handler(__FILE__, __LINE__);

    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
      _Error_Handler(__FILE__, __LINE__);

    hdma2d.Instance->AMTCR = 0x1001; // 16 cycle delay to match the FMC's speed
  }
}

static void DMA2D_R2M_Init(void)
{
  if (dma_mode != DMA2D_R2M) {
    dma_mode = DMA2D_R2M;
    if (HAL_DMA2D_Init(&hdma2d_r2m) != HAL_OK)
      Error_Handler();
    hdma2d_r2m.Instance->CR |= DMA2D_R2M;
    hdma2d_r2m.Instance->AMTCR = 0;
  }
}

void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* dma2dHandle)
{
  if(dma2dHandle->Instance==DMA2D)
  {
    __HAL_RCC_DMA2D_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);   
  }
}

void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* dma2dHandle)
{
  if(dma2dHandle->Instance==DMA2D)
  {
    __HAL_RCC_DMA2D_CLK_DISABLE();
  }
}

void DMA2D_WriteBuffer(uint32_t buffer_p, uint32_t dest_p, uint32_t length) {
  DMA2D_M2M_Init();
  DMA2D->OOR = 0;
  if (HAL_DMA2D_Start_IT(&hdma2d, buffer_p, dest_p, 320, 240) != HAL_OK) {
      Error_Handler();
  }
}

void DMA2D_FillRect(uint32_t color, uint32_t dest_p, uint16_t w, uint16_t h) {
  DMA2D_R2M_Init();
  DMA2D->OOR = 320-w;
  if (HAL_DMA2D_Start_IT(&hdma2d_r2m, color, dest_p, w, h) != HAL_OK) {
    Error_Handler();
  }
}
