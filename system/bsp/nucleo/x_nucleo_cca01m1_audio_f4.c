/**
******************************************************************************
* @file    x_nucleo_cca01m1_audio_f4.c
* @author  Central Labs
* @version V1.0.0
* @date    6-October-2015
* @brief   This file provides a set of functions needed to manage the 
*          sound terminal device on STM32F4.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "x_nucleo_cca01m1_audio_f4.h"

/** @addtogroup BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1_AUDIO
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Defines 
* @{
*/
#ifndef NULL
#define NULL      (void *) 0
#endif

/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_Variables 
* @{
*/
SOUNDTERMINAL_Drv_t *AudioDrv;
SOUNDTERMINAL_Object_t SoundTerminalObjs[SOUNDTERMINAL_DEVICE_NBR];
I2S_HandleTypeDef hAudioOutI2s[SOUNDTERMINAL_DEVICE_NBR];
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Private_FunctionPrototypes 
* @{
*/
static void I2Sx_Init(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq);
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_AUDIO_Exported_Functions 
* @{
*/

/**
* @brief  Initialization of AUDIO Device.
* @param  OutputDevice: device to be initialized
*         This parameter can be a value of @ref SOUNDTERMINAL_devices  
* @param  Volume: initialization volume
* @param  AudioFreq: sampling frequency
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{  
  uint8_t ret = AUDIO_OK;   
  
  if(OutputDevice == SOUNDTERMINAL_DEV1)
  {    
    SoundTerminalObjs[OutputDevice].DevAddr = SOUNDTERMINAL_DEV1_ADDR;
    hAudioOutI2s[OutputDevice].Instance = AUDIO_OUT1_I2S_INSTANCE;
  }
  else if (OutputDevice == SOUNDTERMINAL_DEV2)
  {
    SoundTerminalObjs[OutputDevice].DevAddr = SOUNDTERMINAL_DEV2_ADDR;
    hAudioOutI2s[OutputDevice].Instance = AUDIO_OUT2_I2S_INSTANCE;    
  }
  
  SoundTerminalObjs[OutputDevice].Instance = OutputDevice;
  SoundTerminalObjs[OutputDevice].IO_Init = STA350BW_I2C_Init;
  SoundTerminalObjs[OutputDevice].IO_Read = STA350BW_I2C_Read;
  SoundTerminalObjs[OutputDevice].IO_ReadMulti = STA350BW_I2C_ReadMulti;
  SoundTerminalObjs[OutputDevice].IO_Write = STA350BW_I2C_Write;
  SoundTerminalObjs[OutputDevice].IO_WriteMulti = STA350BW_I2C_WriteMulti;
  SoundTerminalObjs[OutputDevice].IO_Delay = STA350BW_I2C_Delay;
  
  /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */ 
  BSP_AUDIO_OUT_ClockConfig(OutputDevice, &hAudioOutI2s[OutputDevice], AudioFreq, NULL);  
  
  if(HAL_I2S_GetState(&hAudioOutI2s[OutputDevice]) == HAL_I2S_STATE_RESET)
  {
    /* Init the I2S MSP: this __weak function can be redefined by the application*/
    BSP_AUDIO_OUT_MspInit(OutputDevice,&hAudioOutI2s[OutputDevice], NULL);
  }
  
  I2Sx_Init(&hAudioOutI2s[OutputDevice], AudioFreq);  
  
  AudioDrv = &STA350BW_Drv;
  
  uint16_t dummy[16]={0};
  
  HAL_I2S_Transmit_DMA(&hAudioOutI2s[OutputDevice], dummy, 16);
  
  BSP_AUDIO_OUT_Reset(OutputDevice);
  /* Audio init */
  AudioDrv->Init(&SoundTerminalObjs[OutputDevice], Volume, AudioFreq, NULL); 
  
  HAL_I2S_DMAStop(&hAudioOutI2s[OutputDevice]);

  if(OutputDevice == SOUNDTERMINAL_DEV1)
  {    
    HAL_NVIC_SetPriority(AUDIO_OUT1_I2S_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
  }
  else if (OutputDevice == SOUNDTERMINAL_DEV2)
  {
    HAL_NVIC_SetPriority(AUDIO_OUT2_I2S_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ);
  }  
  
  return ret;
}

/**
* @brief  De-initialization of AUDIO Device.
* @param  OutputDevice: device to be de-initialized
*         This parameter can be a value of @ref SOUNDTERMINAL_devices  
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_DeInit(uint16_t OutputDevice)
{  
 HAL_I2S_DMAStop(&hAudioOutI2s[OutputDevice]);
    /* Init the I2S MSP: this __weak function can be redefined by the application*/
 BSP_AUDIO_OUT_MspDeInit(OutputDevice,&hAudioOutI2s[OutputDevice], NULL); 
 return AUDIO_OK;
}


/**
* @brief  Starts audio streaming to the AUDIO Device.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  *pBuffer: pointer to the data to be streamed
* @param  Size: data size
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_Play(uint16_t OutputDevice, uint16_t *pBuffer, uint32_t Size)
{
  /* Call the audio Codec Play function */
  if(AudioDrv->Play(&SoundTerminalObjs[OutputDevice], pBuffer, Size, NULL) != 0)
  {  
    return AUDIO_ERROR;
  }
  else
  {      
    HAL_I2S_Transmit_DMA(&hAudioOutI2s[OutputDevice], (uint16_t *)pBuffer, DMA_MAX(Size/2)); 
    return AUDIO_OK;
  }
}

/**
  * @brief  Sends n-Bytes on the I2S interface.
  * @param  pData: Pointer to data address 
  * @param  Size: Number of data to be written
  */
void BSP_AUDIO_OUT_ChangeBuffer(uint16_t OutputDevice,uint16_t *pData, uint16_t Size)
{
  HAL_I2S_Transmit_DMA(&hAudioOutI2s[OutputDevice], pData, Size); 
}


/**
* @brief  This function Pauses the audio stream. In case
*         of using DMA, the DMA Pause feature is used.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @WARNING When calling BSP_AUDIO_OUT_Pause() function for pause, only
*          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
*          function for resume could lead to unexpected behavior).
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_Pause(uint16_t OutputDevice)
{    
  /* Call the Audio Codec Pause/Resume function */
  if(AudioDrv->Pause(&SoundTerminalObjs[OutputDevice],NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Call the Media layer pause function */
    HAL_I2S_DMAPause(&hAudioOutI2s[OutputDevice]);
    
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
* @brief  Resumes the audio stream.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_Resume(uint16_t OutputDevice)
{    
  /* Call the Audio Codec Pause/Resume function */
  if(AudioDrv->Resume(&SoundTerminalObjs[OutputDevice],NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Call the Media layer pause function */
    HAL_I2S_DMAResume(&hAudioOutI2s[OutputDevice]);
    
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
* @brief  Stop the audio stream.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_Stop(uint16_t OutputDevice)
{ /* Call the Media layer pause function */
	  HAL_I2S_DMAStop(&hAudioOutI2s[OutputDevice]);
	
	
  /* Call the Audio Codec Pause/Resume function */
  if(AudioDrv->Stop(&SoundTerminalObjs[OutputDevice],NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
   
 //  BSP_AUDIO_OUT_Reset(OutputDevice);
    
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
* @brief  Set volume.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  channel: channel to be configured
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_SetVolume(uint16_t OutputDevice, uint8_t channel, uint8_t value)
{
  
  if(AudioDrv->SetVolume(&SoundTerminalObjs[OutputDevice],channel, value, NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;  
}

/**
* @brief  Set Equalization.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  ramBlock: device RAM block to be written
* @param  filterNumber: filter to be used
* @param  filterValues: pointer to filter values    
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
* @note   for specific information about biquadratic filters setup, please refer 
*         to the STA350BW component datasheet, available at 
*         http://www.st.com/web/catalog/sense_power/FM125/SC1756/PF251568?s_searchtype=partnumber
*/
uint8_t BSP_AUDIO_OUT_SetEq(uint16_t OutputDevice, uint8_t ramBlock, uint8_t filterNumber, uint32_t * filterValues)
{    
  if(AudioDrv->SetEq(&SoundTerminalObjs[OutputDevice], ramBlock, filterNumber, filterValues, NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;    
}

/**
* @brief  Set Tone.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_SetTone(uint16_t OutputDevice, uint8_t toneGain)
{
  
  if(AudioDrv->SetTone(&SoundTerminalObjs[OutputDevice],toneGain, NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;  
}

/**
* @brief  Set mute.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  channel: channel to be muted
* @param  state: enable or disable value
*         This parameter can be a value of @ref STA350BW_state_define 
* @param  filterValues: pointer to filter values    
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_SetMute(uint16_t OutputDevice, uint8_t channel, uint8_t state)
{  
  if(AudioDrv->SetMute(&SoundTerminalObjs[OutputDevice], channel, state, NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;      
}

/**
* @brief  Set frequency of the I2S bus.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  AudioFreq: sampling frequency
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_SetFrequency(uint16_t OutputDevice, uint32_t AudioFreq)
{ 

  if(AudioDrv->SetFrequency(&SoundTerminalObjs[OutputDevice], AudioFreq, NULL) != 0)
  {
    return AUDIO_ERROR;
  } 
  
  /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */   
  BSP_AUDIO_OUT_ClockConfig(OutputDevice, &hAudioOutI2s[OutputDevice], AudioFreq, NULL);  
  I2Sx_Init(&hAudioOutI2s[OutputDevice], AudioFreq);  
  
  return AUDIO_OK; 
}

/**
* @brief  Power off the device.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_PowerOff(uint16_t OutputDevice)
{
  if(AudioDrv->PowerOff(&SoundTerminalObjs[OutputDevice], NULL) != 0)
  {
    return AUDIO_ERROR;
  }  
  return AUDIO_OK; 
}

/**
* @brief  Power on the device.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_PowerOn(uint16_t OutputDevice)
{
  
  if(AudioDrv->PowerOn(&SoundTerminalObjs[OutputDevice], NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK; 
  
}

/**
* @brief  Stop playing audio stream 
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_EnterLowPowerMode(uint16_t OutputDevice)
{    
  return AUDIO_OK; 
}

/**
* @brief  Stop playing audio stream 
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_ExitLowPowerMode(uint16_t OutputDevice)
{  
  return AUDIO_OK;   
}

/**
* @brief  This function can be used to set advanced DSP options in order to 
*         use advanced features on the STA350BW device.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  option: specific option to be setted up
*         This parameter can be a value of @ref STA350BW_DSP_option_selection 
* @param  state: state of the option to be controlled. Depending on the selected 
*         DSP feature to be controlled, this value can be either ENABLE/DISABLE 
*         or a specific numerical parameter related to the specific DSP function. 
*         This parameter can be a value of @ref STA350BW_state_define   
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
uint8_t BSP_AUDIO_OUT_SetDSPOption(uint16_t OutputDevice, uint8_t option, uint8_t state)
{  
  
  if(AudioDrv->SetDSPOption(&SoundTerminalObjs[OutputDevice], option, state, NULL) != 0)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;    
}

/**
* @brief  Initializes BSP_AUDIO_OUT MSP.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  hi2s: I2S handle if required
* @param  Params: additional parameters where required
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
__weak uint8_t BSP_AUDIO_OUT_MspInit(uint16_t OutputDevice,I2S_HandleTypeDef *hi2s, void *Params)
{ 
  GPIO_InitTypeDef  gpio_init_structure;    
  
  if(OutputDevice == SOUNDTERMINAL_DEV1)
  {    
    static DMA_HandleTypeDef hdma_i2sTx_1;   
    
    AUDIO_OUT1_I2S_CLK_ENABLE();
    AUDIO_OUT1_I2S_SD_CLK_ENABLE(); 
    AUDIO_OUT1_I2S_SCK_CLK_ENABLE();
    AUDIO_OUT1_I2S_MCK_CLK_ENABLE();
    AUDIO_OUT1_I2S_WS_CLK_ENABLE();
    
    /* I2S3 pins configuration: WS, SCK and SD pins -----------------------------*/
    gpio_init_structure.Pin         = AUDIO_OUT1_I2S_SCK_PIN ; 
    gpio_init_structure.Mode        = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull        = GPIO_PULLDOWN;
    gpio_init_structure.Speed       = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate   = AUDIO_OUT1_I2S_SCK_SD_WS_MCK_AF;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_SCK_GPIO_PORT, &gpio_init_structure);
    
    gpio_init_structure.Pin         = AUDIO_OUT1_I2S_SD_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_SD_GPIO_PORT, &gpio_init_structure);     
    
    gpio_init_structure.Pin         = AUDIO_OUT1_I2S_WS_PIN ;
    HAL_GPIO_Init(AUDIO_OUT1_I2S_WS_GPIO_PORT, &gpio_init_structure); 
    
    gpio_init_structure.Pin         = AUDIO_OUT1_I2S_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT1_I2S_MCK_GPIO_PORT, &gpio_init_structure);   
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT1_I2S_DMAx_CLK_ENABLE(); 
    
    if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */   
      hdma_i2sTx_1.Init.Channel             = AUDIO_OUT1_I2S_DMAx_CHANNEL;  
      hdma_i2sTx_1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_i2sTx_1.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_i2sTx_1.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_i2sTx_1.Init.PeriphDataAlignment = AUDIO_OUT1_I2S_DMAx_PERIPH_DATA_SIZE;
      hdma_i2sTx_1.Init.MemDataAlignment    = AUDIO_OUT1_I2S_DMAx_MEM_DATA_SIZE;
      hdma_i2sTx_1.Init.Mode                = DMA_CIRCULAR;
      hdma_i2sTx_1.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_i2sTx_1.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
      hdma_i2sTx_1.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_i2sTx_1.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_i2sTx_1.Init.PeriphBurst         = DMA_PBURST_SINGLE; 
      
      hdma_i2sTx_1.Instance                 = AUDIO_OUT1_I2S_DMAx_STREAM;
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_1);
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_i2sTx_1);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_i2sTx_1);
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ); 
    }
    
    /*Reset pin configuration*/
    AUDIO_OUT1_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    gpio_init_structure.Pin = AUDIO_OUT1_RST_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT1_RST_GPIO_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);    
    
  }
  else if (OutputDevice == SOUNDTERMINAL_DEV2)
  {   
    static DMA_HandleTypeDef hdma_i2sTx_2;
    
    AUDIO_OUT2_I2S_CLK_ENABLE();
    AUDIO_OUT2_I2S_SD_CLK_ENABLE(); 
    AUDIO_OUT2_I2S_SCK_CLK_ENABLE();
    AUDIO_OUT2_I2S_MCK_CLK_ENABLE();
    AUDIO_OUT2_I2S_WS_CLK_ENABLE();
    
    /* I2S3 pins configuration: WS, SCK and SD pins -----------------------------*/
    gpio_init_structure.Pin         = AUDIO_OUT2_I2S_SCK_PIN ; 
    gpio_init_structure.Mode        = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull        = GPIO_PULLDOWN;
    gpio_init_structure.Speed       = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate   = AUDIO_OUT2_I2S_SCK_SD_WS_MCK_AF;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_SCK_GPIO_PORT, &gpio_init_structure);
    
    gpio_init_structure.Pin         = AUDIO_OUT2_I2S_SD_PIN ;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_SD_GPIO_PORT, &gpio_init_structure);     
    
    gpio_init_structure.Pin         = AUDIO_OUT2_I2S_WS_PIN ;
    HAL_GPIO_Init(AUDIO_OUT2_I2S_WS_GPIO_PORT, &gpio_init_structure); 
    
    gpio_init_structure.Pin         = AUDIO_OUT2_I2S_MCK_PIN; 
    HAL_GPIO_Init(AUDIO_OUT2_I2S_MCK_GPIO_PORT, &gpio_init_structure);   
    
    /* Enable the I2S DMA clock */
    AUDIO_OUT2_I2S_DMAx_CLK_ENABLE(); 
    
    if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
    {
      /* Configure the hdma_i2sTx handle parameters */   
      hdma_i2sTx_2.Init.Channel             = AUDIO_OUT2_I2S_DMAx_CHANNEL;  
      hdma_i2sTx_2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_i2sTx_2.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_i2sTx_2.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_i2sTx_2.Init.PeriphDataAlignment = AUDIO_OUT2_I2S_DMAx_PERIPH_DATA_SIZE;
      hdma_i2sTx_2.Init.MemDataAlignment    = AUDIO_OUT2_I2S_DMAx_MEM_DATA_SIZE;
      hdma_i2sTx_2.Init.Mode                = DMA_CIRCULAR;
      hdma_i2sTx_2.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_i2sTx_2.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
      hdma_i2sTx_2.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_i2sTx_2.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_i2sTx_2.Init.PeriphBurst         = DMA_PBURST_SINGLE;       
      hdma_i2sTx_2.Instance                 = AUDIO_OUT2_I2S_DMAx_STREAM;
      
      /* Associate the DMA handle */
      __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_2);
      
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(&hdma_i2sTx_2);
      
      /* Configure the DMA Stream */
      HAL_DMA_Init(&hdma_i2sTx_2); 
      
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ);       
    }
    
    /*Reset pin configuration*/
    AUDIO_OUT2_RST_GPIO_CLK_ENABLE();
    
    /* Configure GPIO PINs to detect Interrupts */
    gpio_init_structure.Pin = AUDIO_OUT2_RST_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(AUDIO_OUT2_RST_GPIO_PORT, &gpio_init_structure);
    HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_SET);    
  }
 else
  {
    return AUDIO_ERROR;
  }  
  return AUDIO_OK;
}

/**
* @brief  Deinitializes I2S MSP.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  hi2s: I2S handle if required
* @param  Params: additional parameters where required
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
__weak uint8_t BSP_AUDIO_OUT_MspDeInit(uint16_t OutputDevice,I2S_HandleTypeDef *hi2s, void *Params)
{
 
   if(OutputDevice == SOUNDTERMINAL_DEV1)
  {       
    AUDIO_OUT1_I2S_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/

    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SCK_GPIO_PORT, AUDIO_OUT1_I2S_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SD_GPIO_PORT, AUDIO_OUT1_I2S_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_WS_GPIO_PORT, AUDIO_OUT1_I2S_WS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT1_I2S_MCK_GPIO_PORT, AUDIO_OUT1_I2S_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN); 
    
    if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hi2s->hdmatx); 

      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ); 
    }    
  }
  else if (OutputDevice == SOUNDTERMINAL_DEV2)
  {   
    AUDIO_OUT1_I2S_CLK_DISABLE();  
    
    /* I2S3 pins de initialization: WS, SCK and SD pins -----------------------------*/

    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_SCK_GPIO_PORT, AUDIO_OUT2_I2S_SCK_PIN);  
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_SD_GPIO_PORT, AUDIO_OUT2_I2S_SD_PIN);    
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_WS_GPIO_PORT, AUDIO_OUT2_I2S_WS_PIN);
    HAL_GPIO_DeInit(AUDIO_OUT2_I2S_MCK_GPIO_PORT, AUDIO_OUT2_I2S_MCK_PIN);   
    HAL_GPIO_DeInit(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN); 
    
    if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
    {     
      /* Deinitialize the Stream for new transfer */
      HAL_DMA_DeInit(hi2s->hdmatx); 

      /* I2S DMA IRQ Channel configuration */
      HAL_NVIC_DisableIRQ(AUDIO_OUT2_I2S_DMAx_IRQ); 
    }    
    } 
 else 
  {
    return AUDIO_ERROR;
    }  	
  
  return AUDIO_OK;
  /* GPIO pins clock and DMA clock can be shut down in the application 
  by surcharging this __weak function */ 
}

/**
* @brief  Clock Config.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @param  hi2s: I2S handle if required
* @param  Params: additional parameters where required
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
*         Being __weak it can be overwritten by the application     
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
__weak uint8_t BSP_AUDIO_OUT_ClockConfig(uint16_t OutputDevice,I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params)
{ 
  RCC_PeriphCLKInitTypeDef rccclkinit;
  
  /* Enable PLLI2S clock */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);

  if (AudioFreq == 96000) 
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192)*/
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 344;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (AudioFreq == 48000) 
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192)*/
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (AudioFreq == 44100)
  {
    /* Other Frequency (11.025/22.500/44.100) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 271;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else if (AudioFreq == 32000)
  {
    /* Other Frequency (11.025/22.500/44.100) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 213;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    return AUDIO_ERROR;
  }  
  return AUDIO_OK;
}

/**
* @brief  Reset the device.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise 
* @retval None
*/
uint8_t BSP_AUDIO_OUT_Reset(uint16_t OutputDevice)
{  
  if(OutputDevice == SOUNDTERMINAL_DEV1)
  {
    HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN, GPIO_PIN_SET);  
  }
  else if (OutputDevice == SOUNDTERMINAL_DEV2)
  {
    HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(AUDIO_OUT2_RST_GPIO_PORT, AUDIO_OUT2_RST_PIN, GPIO_PIN_SET);    
  }
  return AUDIO_OK; 
}

/**
* @brief Tx Transfer completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Call the record update function to get the next buffer to fill and its size (size is ignored) */
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    BSP_AUDIO_OUT_TransferComplete_CallBack(SOUNDTERMINAL_DEV1);
  }
  if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
  {
    BSP_AUDIO_OUT_TransferComplete_CallBack(SOUNDTERMINAL_DEV2);
  }  
}

/**
* @brief Tx Transfer Half completed callbacks
* @param hi2s: I2S handle
* @retval None
*/
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the remaining file size and new address offset: This function 
  should be coded by user (its prototype is already declared in stm324xg_eval_audio.h) */  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    BSP_AUDIO_OUT_HalfTransfer_CallBack(SOUNDTERMINAL_DEV1);
  }
  if(hi2s->Instance == AUDIO_OUT2_I2S_INSTANCE)
  {
    BSP_AUDIO_OUT_HalfTransfer_CallBack(SOUNDTERMINAL_DEV2);
  }
}

/**
* @brief  Manages the DMA full Transfer complete event.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval None
*/
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(uint16_t OutputDevice)
{
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  OutputDevice: device to be used
*         This parameter can be a value of @ref SOUNDTERMINAL_devices 
* @retval None
*/
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint16_t OutputDevice)
{ 
}

/**
* @brief  Initializes the Audio Codec audio interface (I2S)
* @note   This function assumes that the I2S input clock (through PLL_R in 
*         Devices RevA/Z and through dedicated PLLI2S_R in Devices RevB/Y)
*         is already configured and ready to be used.    
* @param  AudioFreq: Audio frequency to be configured for the I2S peripheral. 
* @retval None
*/
static void I2Sx_Init(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq)
{
  /* Disable I2S block */
  __HAL_I2S_DISABLE(hi2s);
  
  /* I2S2 peripheral configuration */
  hi2s->Init.AudioFreq    = AudioFreq;
  hi2s->Init.ClockSource  = I2S_CPOL_LOW;
  hi2s->Init.CPOL         = I2S_CPOL_LOW;
  hi2s->Init.DataFormat   = I2S_DATAFORMAT_16B;
  hi2s->Init.MCLKOutput   = I2S_MCLKOUTPUT_ENABLE;
  hi2s->Init.Mode         = I2S_MODE_MASTER_TX;
  hi2s->Init.Standard     = I2S_STANDARD_PHILIPS;
  
  /* Init the I2S */
  HAL_I2S_Init(hi2s); 
  
  /* Disable I2S block */
  __HAL_I2S_ENABLE(hi2s);  
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
