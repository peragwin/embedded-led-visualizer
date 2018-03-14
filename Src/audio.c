// Modula for processing audio

#include "audio.h"
#include "i2s.h"
#include "sai.h"
#include "gpio.h"
#include "fastLog.h"
#include "bucketer.h"

static int32_t audio_buffer[2 * AUDIO_BUFFER_SIZE] = {0};

static void init_fft(uint16_t size);
static void init_scaled_window(uint16_t size);
static void init_bucketer(uint16_t size);
static void init_frequency_sensor(uint16_t size, uint16_t columns);
static void process(int16_t *audio);

static void audioTransferComplete(DMA_HandleTypeDef *hdma);
static void audioTransferError(DMA_HandleTypeDef *hdma);

void Audio_Init(void) {
  SAI_HandleTypeDef *hsai = &hsai_BlockA1;
  uint32_t buffer_0 = (uint32_t)audio_buffer;
  uint32_t buffer_1 = (uint32_t)(audio_buffer + AUDIO_BUFFER_SIZE);

  hdma_sai1_a.XferCpltCallback = audioTransferComplete;
  hdma_sai1_a.XferErrorCallback = audioTransferError;

  // HAL_SAI_Receive_DMA
  if(hsai->State == HAL_SAI_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsai);

    //hsai->pBuffPtr = buffer_0;
    hsai->XferSize = AUDIO_BUFFER_SIZE;
    hsai->XferCount = AUDIO_BUFFER_SIZE;
    hsai->ErrorCode = HAL_SAI_ERROR_NONE;
    hsai->State = HAL_SAI_STATE_BUSY_RX;

    /* Enable the Rx DMA Stream */
    if (HAL_DMAEx_MultiBufferStart_IT(&hdma_sai1_a,
      (uint32_t)&hsai->Instance->DR, buffer_0, buffer_1, hsai->XferSize) != HAL_OK)
      Error_Handler();

    /* Check if the SAI is already enabled */
    if((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == RESET)
    {
      /* Enable SAI peripheral */
      __HAL_SAI_ENABLE(hsai);
    }

    /* Enable the interrupts for error handling */
    //__HAL_SAI_ENABLE_IT(hsai, SAI_InterruptFlag(hsai, SAI_MODE_DMA));

    /* Enable SAI Rx DMA Request */
    hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

    /* Process Unlocked */
    __HAL_UNLOCK(hsai);

  }
  else
  {
    Error_Handler();
  }

  // I2S_HandleTypeDef *hi2s = &hi2s1;
  // uint32_t buffer_0 = (uint32_t)audio_buffer;
  // uint32_t buffer_1 = (uint32_t)(audio_buffer + AUDIO_BUFFER_SIZE);

  // hdma_spi1_rx.XferCpltCallback = audioTransferComplete;
  // hdma_spi1_rx.XferErrorCallback = audioTransferError;

  // if(hi2s->State == HAL_I2S_STATE_READY)
  // {    
  //   if(((hi2s->Instance->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)) == I2S_DATAFORMAT_24B)||\
  //     ((hi2s->Instance->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN)) == I2S_DATAFORMAT_32B))
  //   {
  //     hi2s->RxXferSize = (AUDIO_BUFFER_SIZE << 1);
  //     hi2s->RxXferCount = (AUDIO_BUFFER_SIZE << 1);
  //   }  
  //   else
  //   {
  //     hi2s->RxXferSize = AUDIO_BUFFER_SIZE;
  //     hi2s->RxXferCount = AUDIO_BUFFER_SIZE;
  //   }

  //   /* Process Locked */
  //   __HAL_LOCK(hi2s);
    
  //   hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
  //   hi2s->State = HAL_I2S_STATE_BUSY_RX;

  //   /* Enable the Rx DMA Channel */      
  //   if (HAL_DMAEx_MultiBufferStart_IT(&hdma_spi1_rx,
  //     (uint32_t)&hi2s->Instance->DR, buffer_0, buffer_1, hi2s->RxXferSize) != HAL_OK)
  //     Error_Handler();
    
  //   /* Check if the I2S is already enabled */ 
  //   if((hi2s->Instance->I2SCFGR &SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE)
  //   {
  //     /* Enable I2S peripheral */    
  //     __HAL_I2S_ENABLE(hi2s);
  //   }
    
  //   /* Enable Rx DMA Request */  
  //   hi2s->Instance->CR2 |= SPI_CR2_RXDMAEN;
    
  //   /* Process Unlocked */
  //   __HAL_UNLOCK(hi2s);
  // }
  // else
  // {
  //   Error_Handler();
  // }

  uint16_t frame_size = AUDIO_BUFFER_SIZE / 2; //hi2s->RxXferSize / 2;
  init_scaled_window(frame_size);
  init_fft(frame_size);
  // low pass to 20khz since thats the cutoff of the microphone
  int bucket_frame_size = (frame_size - (4 * frame_size / 24)) / 2;
  init_bucketer(bucket_frame_size);
  init_frequency_sensor(NUM_BUCKETS, NUM_COLUMNS);
}

void Audio_ensure_i2s_frame_sync(void) {
  if (hi2s1.Instance->SR & 0x100) {
    __HAL_I2S_DISABLE(&hi2s1);
    LED_Set(LED_RED, 1);

    HAL_Delay(1);

    __HAL_I2S_ENABLE(&hi2s1);
    LED_Set(LED_RED, 0);
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

int32_t* Audio_GetCurrentBuffer(void) {
  return Audio_GetBuffer(audio_buffer_toggle);
}

// Returns the part of the buffer that isn't currently being operated on:
// if operating on 1 -> &audio_buffer[0]
// else            0 -> &audio_buffer[AUDIO_BUFFER_SIZE]
int32_t* Audio_GetBuffer(uint8_t which) {
  return which ? audio_buffer : (audio_buffer + AUDIO_BUFFER_SIZE);
}

float32_t scaled_window[AUDIO_BUFFER_SIZE / 2];

static void init_scaled_window(uint16_t size) {
  for (int i = 0; i < size; i++) {
    // blackman-harris window
    float32_t c1 = arm_cos_f32(2.0 * PI * (float32_t)i / (float32_t)(size - 1));
    float32_t c2 = arm_cos_f32(4.0 * PI * (float32_t)i / (float32_t)(size - 1));
    float32_t c3 = arm_cos_f32(6.0 * PI * (float32_t)i / (float32_t)(size - 1));
    scaled_window[i] = 0.35875 - 0.48829 * c1 + 0.14128 * c2 - 0.01168 * c3;
    scaled_window[i] /= (float32_t)(1<<16);//32768; // 2**15 to get a range of [-1, 1]
  }
}

arm_rfft_fast_instance_f32 fft_struct;

static void init_fft(uint16_t size) {
  arm_rfft_fast_init_f32(&fft_struct, size);
}

Bucketer_TypeDef *bucketer;

static void init_bucketer(uint16_t size) {
  bucketer = NewBucketer(size, NUM_BUCKETS, 32, 16000);
}

FrequencySensor_TypeDef *frequency_sensor;

static void init_frequency_sensor(uint16_t size, uint16_t columns) {
  frequency_sensor = NewFrequencySensor(size, columns);
}

Drivers_TypeDef* Audio_GetProcessedOutput(void) {
  return frequency_sensor->drivers;
}

static void average_stereo(int16_t *dest, int16_t *src, int length) {
  for (int i = 0; i < length/2; i++) {
    dest[i] = (src[2*i] + src[2*i+1]) / 2;
  }
}

static void convert_to_float(float32_t *dest, int16_t *src, int length) {
  int i = length;
  while (i--) {
    // The left + right shift is needed to fix the 2's complement that is not
    // registered when DMA does a 32bit transfer of the 24bit value.
    dest[i] = (float32_t)((src[i] << 8) >> 8);
  }
}

static void scale_and_apply_window(float32_t *frame, int length) {
  arm_mult_f32(frame, scaled_window, frame, length);
}

static void power_spectrum(float32_t *dest, float32_t *src) {
  uint16_t fft_size = fft_struct.fftLenRFFT;
  arm_rfft_fast_f32(&fft_struct, src, dest, 0);
  arm_abs_f32(dest, dest, fft_size);
  for (int i = 0; i < fft_size; i++) {
    dest[i] = log2_2521(1 + dest[i]);
  }
}

volatile int processing = 0;

static void process(int16_t *audio) {
  if (processing) {
    LED_Set(LED_RED, 1);
  } else {
    LED_Set(LED_RED, 0);
  }
  processing = 1;
  LED_Set(LED_BLUE, 1);
  int frame_size = AUDIO_BUFFER_SIZE / 2;
  // int fft_size = frame_size / 2;
  int16_t mono[frame_size];
  float32_t frame[frame_size];
  float32_t fft_frame[frame_size];

  average_stereo(mono, audio, AUDIO_BUFFER_SIZE);
  convert_to_float(frame, mono, frame_size);
  scale_and_apply_window(frame, frame_size);

  power_spectrum(fft_frame, frame);

  Bucket(bucketer, fft_frame+1); // ignore DC component

  FS_Process(frequency_sensor, bucketer->output);
  LED_Set(LED_BLUE, 0);
  processing = 0;
}