#ifndef __dma2d_H
#define __dma2d_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

extern DMA2D_HandleTypeDef hdma2d;

extern void _Error_Handler(char *, int);

void MX_DMA2D_Init(void);

void DMA2D_WriteBuffer(uint32_t buffer_p, uint32_t dest_p, uint32_t length);

#ifdef __cplusplus
}
#endif
#endif /*__ dma2d_H */