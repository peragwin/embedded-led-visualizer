#ifndef __dma2d_H
#define __dma2d_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

extern DMA2D_HandleTypeDef hdma2d;
extern DMA2D_HandleTypeDef hdma2d_r2m;

extern void _Error_Handler(char *, int);

void MX_DMA2D_Init(void);

void DMA2D_WriteBuffer(uint32_t buffer_p, uint32_t dest_p, uint32_t length);
void DMA2D_FillRect(uint32_t color, uint32_t dest_p, uint16_t w, uint16_t h);

#ifdef __cplusplus
}
#endif
#endif /*__ dma2d_H */