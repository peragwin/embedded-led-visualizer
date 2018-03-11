// Modula for processing audio

#include "audio.h"
#include "i2s.h"

static int16_t audio_buffer[2 * AUDIO_BUFFER_SIZE] = {0};

static void audioTransferComplete(DMA_HandleTypeDef *hdma);
static void audioTransferError(DMA_HandleTypeDef *hdma);
static void process(int16_t *audio);

void Audio_Init(void) {
  I2S_HandleTypeDef *hi2s = &hi2s1;
  uint32_t buffer_0 = (uint32_t)audio_buffer;
  uint32_t buffer_1 = (uint32_t)(audio_buffer + AUDIO_BUFFER_SIZE);

  hdma_spi1_rx.XferCpltCallback = audioTransferComplete;
  hdma_spi1_rx.XferErrorCallback = audioTransferError;
    
  if(hi2s->State == HAL_I2S_STATE_READY)
  {    
    if(((hi2s->Instance->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)) == I2S_DATAFORMAT_24B)||\
      ((hi2s->Instance->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)) == I2S_DATAFORMAT_32B))
    {
      hi2s->RxXferSize = (AUDIO_BUFFER_SIZE << 1);
      hi2s->RxXferCount = (AUDIO_BUFFER_SIZE << 1);
    }  
    else
    {
      hi2s->RxXferSize = AUDIO_BUFFER_SIZE;
      hi2s->RxXferCount = AUDIO_BUFFER_SIZE;
    }
    /* Process Locked */
    __HAL_LOCK(hi2s);
    
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State = HAL_I2S_STATE_BUSY_RX;

    /* Enable the Rx DMA Channel */      
    if (HAL_DMAEx_MultiBufferStart_IT(&hdma_spi1_rx,
      (uint32_t)&hi2s->Instance->DR, buffer_0, buffer_1, hi2s->RxXferSize) != HAL_OK)
      Error_Handler();
    
    /* Check if the I2S is already enabled */ 
    if((hi2s->Instance->I2SCFGR &SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE)
    {
      /* Enable I2S peripheral */    
      __HAL_I2S_ENABLE(hi2s);
    }
    
    /* Enable Rx DMA Request */  
    hi2s->Instance->CR2 |= SPI_CR2_RXDMAEN;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);
  }
  else
  {
    Error_Handler();
  }
}

int audio_buffer_toggle;

// AudioTransferComplete moves the pointer for the memory block which just finished transfering
// to the next location.
static void audioTransferComplete(DMA_HandleTypeDef *hdma)
{
  process(Audio_GetBuffer(audio_buffer_toggle ^= 1));
  // LED_Set(LED_RED, audio_buffer_toggle ^= 1);
}

static void audioTransferError(DMA_HandleTypeDef *hdma) {
  Error_Handler();
}

int16_t* Audio_GetCurrentBuffer(void) {
  return Audio_GetBuffer(audio_buffer_toggle);
}

// Returns the part of the buffer that isn't currently being operated on:
// if operating on 1 -> &audio_buffer[0]
// else            0 -> &audio_buffer[AUDIO_BUFFER_SIZE]
int16_t* Audio_GetBuffer(uint8_t which) {
  return which ? audio_buffer : (audio_buffer + AUDIO_BUFFER_SIZE);
}

static void process(int16_t *audio) {
  return;
}