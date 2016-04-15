/**
******************************************************************************
* @file    audio_application.h 
* @author  Central Labs
* @version V 0.1
* @date    18-August-2015
* @brief   Header for audio_application.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_APPLICATION_H
#define __AUDIO_APPLICATION_H

/* Includes ------------------------------------------------------------------*/
//#include "cube_hal.h"
#include "x_nucleo_cca01m1_audio_f4.h"
#include "Fragment1.h"
#include "BiquadCalculator.h"
#include "stdlib.h"


/** @addtogroup X_NUCLEO_CCA01M1_Applications
* @{
*/ 

/** @addtogroup Audio_Streaming
* @{
*/

/** @defgroup AUDIO_APPLICATION 
* @{
*/


/** @defgroup AUDIO_APPLICATION_Private_Types 
* @{
*/  

/**
* @}
*/ 


/** @defgroup AUDIO_APPLICATION_Exported_Defines 
* @{
*/
#define AUDIO_OUTPUT_BUFF_SIZE 512              /* Output buffer size for each channel */
#define DEFAULT_SAMPLING_FREQUENCY 32000        /* Default Sampling frequency */
#define DEFAULT_VOLUME 0x22                     /* Default Volume */
#define FILTER_NB 2

/**
* @}
*/


/** @defgroup AUDIO_APPLICATION_Exported_Functions_Prototypes 
* @{
*/
uint32_t Init_Biquads_Filter(void);
uint32_t Init_AudioOut_Device(void);
uint32_t Start_AudioOut_Device(void);
uint32_t Stop_AudioOut_Device(void);
uint32_t Switch_Demo(void);


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



#endif /* __AUDIO_APPLICATION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
