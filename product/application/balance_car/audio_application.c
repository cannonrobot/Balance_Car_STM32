/**
******************************************************************************
* @file    audio_application.c 
* @author  Central Labs
* @version V 1.0
* @date    18-August-2015
* @brief   Audio input application. 
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
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
********************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "audio_application.h"
#include "BiquadPresets.h"

/** @addtogroup X_NUCLEO_CCA01M1_Applications
* @{
*/ 

/** @addtogroup Audio_Streaming
* @{
*/

/** @defgroup AUDIO_APPLICATION 
* @{
*/

/** @defgroup AUDIO_APPLICATION_Exported_Variables 
* @{
*/

/**
* @}
*/

/** @defgroup AUDIO_APPLICATION_Private_Variables 
* @{
*/
#define DEMO_NUMBER 5
static int16_t Audio_output_buffer[AUDIO_OUTPUT_BUFF_SIZE * 2];
static uint32_t song_position = 0;
/**
* @}
*/

/** @defgroup AUDIO_APPLICATION_Exported_Function 
* @{
*/

/**
* @brief  Initializes all the required peripherals using the BSP Init function.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Init_AudioOut_Device(void)
{
  return BSP_AUDIO_OUT_Init(SOUNDTERMINAL_DEV1,DEFAULT_VOLUME,DEFAULT_SAMPLING_FREQUENCY);  
}

/**
* @brief  Starts audio output.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Start_AudioOut_Device(void)
{
  return BSP_AUDIO_OUT_Play(SOUNDTERMINAL_DEV1, (uint16_t *)Audio_output_buffer, AUDIO_OUTPUT_BUFF_SIZE*2);
}

/**
* @brief  Stops audio output.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Stop_AudioOut_Device(void)
{
  return BSP_AUDIO_OUT_Stop(SOUNDTERMINAL_DEV1);
}

/**
* @brief  Switch Filter configuration for demo purpose.
* @param  None
* @retval AUDIO_OK if no problem during initialization, AUDIO_ERROR otherwise
*/
uint32_t Switch_Demo(void)
{  
  BIQUAD_Filter_t Biquad_filter;
  uint8_t ret = 0;    
  static uint8_t current_demo = 0;
  
  switch(current_demo)
  {
  case 0:
    { 
      /*Setup Default Master Volume (in case this has been change during demos*/
      BSP_AUDIO_OUT_SetVolume(SOUNDTERMINAL_DEV1,  STA350BW_CHANNEL_MASTER ,DEFAULT_VOLUME);      
      
      /*Setup a Second Order High Pass with Fc = 1 KHz using the first biq filter for each channel. Coefficients are stored in BANK 1*/
      /*When coefficients are written to RAM, EQ is automatically activated
      It's possible to switch between different pre-stores presets switching RAM banks using SetDSPOptions with the STA350BW_RAM_BANK_SELECT
      parameter, for example:
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SELECT,STA350BW_RAM_BANK_FIRST);*/
      
      Biquad_filter.Type = BIQUAD_CALCULATOR_SO_HPF;
      Biquad_filter.Fs = DEFAULT_SAMPLING_FREQUENCY;
      Biquad_filter.Fc = 1000;
      Biquad_filter.Q = 0.80;
      Biquad_filter.Slope = 0; /*Not used for this kind of filter*/
      Biquad_filter.Gain = 0; /*Not used for this kind of filter*/    
      BQ_CALC_ComputeFilter(&Biquad_filter);      
      BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_FIRST, STA350BW_CH1_BQ1, Biquad_filter.Coefficients);
      BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_FIRST, STA350BW_CH2_BQ1, Biquad_filter.Coefficients); 
      
      /*Remove EQ and Tone bypass  (in case this has been change during demos)*/
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C1EQBP, STA350BW_DISABLE);
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C2EQBP, STA350BW_DISABLE);     
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C1TCB, STA350BW_DISABLE);
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C2TCB, STA350BW_DISABLE);
      
      break;
    }
  case 1:
    {      
      /*Setup a preset for Bass Boost using 4 biquads for each channel. Coefficients are stored in BANK 2*/
      /*When coefficients are written to RAM, EQ is automatically activated
      It's possible to switch between different pre-stores presets switching RAM banks using SetDSPOptions with the STA350BW_RAM_BANK_SELECT
      parameter, for example:
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SELECT,STA350BW_RAM_BANK_FIRST);*/
      
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH1_BQ1, (uint32_t *)&BASS_BOOST2_EQ_PRESET[0]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH1_BQ2, (uint32_t *)&BASS_BOOST2_EQ_PRESET[5]);  
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH1_BQ3, (uint32_t *)&BASS_BOOST2_EQ_PRESET[10]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH1_BQ4, (uint32_t *)&BASS_BOOST2_EQ_PRESET[15]);
      
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH2_BQ1, (uint32_t *)&BASS_BOOST2_EQ_PRESET[0]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH2_BQ2, (uint32_t *)&BASS_BOOST2_EQ_PRESET[5]);  
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH2_BQ3, (uint32_t *)&BASS_BOOST2_EQ_PRESET[10]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SECOND, STA350BW_CH2_BQ4, (uint32_t *)&BASS_BOOST2_EQ_PRESET[15]);    
      
      break; 
    }    
  case 2:
    {
      /*Setup a preset for Vocal using 4 biquads for each channel. Coefficients are stored in BANK 3*/
      /*When coefficients are written to RAM, EQ is automatically activated
      It's possible to switch between different pre-stores presets switching RAM banks using SetDSPOptions with the STA350BW_RAM_BANK_SELECT
      parameter, for example:
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_SELECT,STA350BW_RAM_BANK_FIRST);*/
      
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH1_BQ1, (uint32_t *)&VOCAL_EQ_PRESET[0]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH1_BQ2, (uint32_t *)&VOCAL_EQ_PRESET[5]);  
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH1_BQ3, (uint32_t *)&VOCAL_EQ_PRESET[10]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH1_BQ4, (uint32_t *)&VOCAL_EQ_PRESET[15]);
      
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH2_BQ1, (uint32_t *)&VOCAL_EQ_PRESET[0]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH2_BQ2, (uint32_t *)&VOCAL_EQ_PRESET[5]);  
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH2_BQ3, (uint32_t *)&VOCAL_EQ_PRESET[10]);
      ret = BSP_AUDIO_OUT_SetEq(SOUNDTERMINAL_DEV1, STA350BW_RAM_BANK_THIRD, STA350BW_CH2_BQ4, (uint32_t *)&VOCAL_EQ_PRESET[15]);
      
      break; 
    }     
  case 3:
    {
      /*Bypass BIQ Filters for both channels*/ 
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C1EQBP,STA350BW_ENABLE);
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C2EQBP,STA350BW_ENABLE);
      
      /*Change tone settings*/
      BSP_AUDIO_OUT_SetTone(SOUNDTERMINAL_DEV1, 0xF0);  
      
      break; 
    }  
  case 4:
    {
      /*Bypass tone control for both channels*/
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C1TCB,STA350BW_ENABLE);
      BSP_AUDIO_OUT_SetDSPOption(SOUNDTERMINAL_DEV1, STA350BW_C2TCB,STA350BW_ENABLE);
      
      /*Modify master volume*/
      BSP_AUDIO_OUT_SetVolume(SOUNDTERMINAL_DEV1,  STA350BW_CHANNEL_MASTER ,0X30);      
      break; 
    }  
  }
  current_demo = (current_demo + 1) % DEMO_NUMBER;
  return ret;
}
#ifdef USE_TEST
/**
* @brief  Manages the DMA Half Transfer complete event.
* @param  OutputDevice: the sound terminal device related to the DMA 
*         channel that generates the interrupt
* @retval None
*/
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint16_t OutputDevice)
{ 
  uint32_t i = 0;
#ifndef USE_STM32L0XX_NUCLEO

  /*Copy song fragment to Audio Output buffer*/
  for(i=0; i<AUDIO_OUTPUT_BUFF_SIZE/2; i++){    
    Audio_output_buffer[2*i]= Fragment1[song_position]; /*Left Channel*/
    Audio_output_buffer[2*i + 1]= Fragment1[song_position]; /*Right Channel*/
    song_position = (song_position +1) % Fragment1_size;
  }
#else
  /*Generate White Noise for the Output buffer*/
   for(i=0; i<AUDIO_OUTPUT_BUFF_SIZE/2; i++)
  {    
  Audio_output_buffer[2*i]= (rand() % 65536) - 32768;
  Audio_output_buffer[2*i + 1]=  Audio_output_buffer[2*i];
}
#endif
  
}

/**
* @brief  Manages the DMA Transfer complete event.
* @param  OutputDevice: the sound terminal device related to the DMA 
*         channel that generates the interrupt
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint16_t OutputDevice)
{

  uint32_t i = 0;
#ifndef USE_STM32L0XX_NUCLEO
  /*Copy song fragment to Audio Output buffer*/
  for(i=AUDIO_OUTPUT_BUFF_SIZE/2; i<AUDIO_OUTPUT_BUFF_SIZE; i++){    
    Audio_output_buffer[2*i]= Fragment1[song_position]; /*Left Channel*/
    Audio_output_buffer[2*i + 1]= Fragment1[song_position]; /*Right Channel*/
    song_position = (song_position +1) % Fragment1_size;
  }  
#else
  /*Generate White Noise for the Output buffer*/       
  for(i=AUDIO_OUTPUT_BUFF_SIZE/2; i<AUDIO_OUTPUT_BUFF_SIZE; i++){
  Audio_output_buffer[2*i]= (rand() % 65536) - 32768; 
  Audio_output_buffer[2*i + 1]=  Audio_output_buffer[2*i];
}
#endif

}

#endif

/**
* @brief EXTI line detection callbacks
* @param GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
 // if(GPIO_Pin == KEY_BUTTON_PIN)
//  {
    /* Toggle LED3 */
 //   BSP_LED_Toggle(LED2);
  //  Switch_Demo();    
 // }
//}
/**
* @}
*/

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
