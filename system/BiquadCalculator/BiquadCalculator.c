/**
******************************************************************************
* @file    BiquadCalculator.c
* @author  Central Labs
* @version V1.0.0
* @date    30-June-2015
* @brief   This file provides a set of functions needed to compute Sound Terminal 
*          biquadratic filters coefficients 
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "BiquadCalculator.h"

#ifndef NULL
#define NULL      (void *) 0
#endif

/** @addtogroup MIDDLEWARES
* @{
*/

/** @addtogroup SOUND_TERMINAL_BIQUAD_CALCULATOR
* @{
*/

/** @defgroup SOUND_TERMINAL_BIQUAD_CALCULATOR_Functions 
* @{
*/

/**
* @brief        function for computing filter coefficient basing on 
*               the filter parameters in the BIQUAD_Filter_t. Computed values 
*               will be saved in the dedicated memory space inside the BIQUAD_Filter_t
*               structure in order to be used to setup the device.
* @param        *pEq: pointer to an istance of BIQUAD_Filter_t.
* @retval       BIQUAD_CALCULATOR_ERROR if an error accurred. In any other case, 
*               the return value is related to the range of the computed values.
*               This parameter can be a value of 
*               ref Biquad_Calculator_Return_values_definition
*/
int32_t BQ_CALC_ComputeFilter(BIQUAD_Filter_t *pEq)
{  
  
  float omega_c = 0, K = 0, W = 0, DE = 0, alpha = 0, beta = 0, a0 = 0, a1 = 0, a2 = 0,
  b0 = 0, b1 = 0, b2 = 0;
  float CoeffSTA350BW[K_NUM];
  int32_t ret = 0;
  
  if(pEq->Fs != 96000)
  {
    omega_c = 2.0f * PI * (float) (pEq->Fc) / (float) (pEq->Fs * 2);
  }
  else
  {
    omega_c = 2.0f * PI * (float) (pEq->Fc) / (float) (pEq->Fs);
    
  }
  K = tanf(omega_c / 2.0f);
  alpha = 1 + K;
  W = K * K;
  DE = 1.0f + (1 / pEq->Q) * K + W;
  
  switch (pEq->Type) {
  case BIQUAD_CALCULATOR_FO_LPF:
    {
      a0 = 1;
      a1 = -((1.0f - K) / alpha);
      b0 = K / alpha;
      b1 = K / alpha;
      a2 = 0;
      b2 = 0;
      break;
    }
  case BIQUAD_CALCULATOR_FO_HPF:
    {
      a0 = 1;
      a1 = -((1.0f - K) / alpha);
      b0 = 1 / alpha;
      b1 = -b0;
      a2 = 0;
      b2 = 0;
      break;
    }
  case BIQUAD_CALCULATOR_SO_LPF:
    {
      a0 = 1.0f;
      a1 = 2.0f * (W - 1) / DE;
      a2 = ((1 - (K / pEq->Q) + W) / DE);
      b0 = W / DE;
      b1 = 2.0f * b0;
      b2 = b0;
      break;
    }
  case BIQUAD_CALCULATOR_SO_HPF:
    {
      a0 = 1.0f;
      a1 = 2.0f * (W - 1) / DE;
      a2 = ((1 - (K / pEq->Q) + W) / DE);
      b0 = 1.0f / DE;
      b1 = -2.0f / DE;
      b2 = b0;
      break;
    }
  case (BIQUAD_CALCULATOR_PEAK):
    {
      float gain = expf((float)(pEq->Gain) * 0.115129254f);
      if (gain < 1.0f)
      {
        float cutValue = 1.0f + K * (1/gain/pEq->Q) + W;
        a0 = 1.0f;
        a1 = 2.0f * (W - 1) / cutValue;
        a2 = ((1 - (1/gain/pEq->Q) * K + W) / cutValue);
        b0 = (1 + K/pEq->Q + W) / cutValue;
        b1 = (2.0f * (W - 1)) / cutValue;
        b2 = (1 - K/pEq->Q + W) / cutValue;
      }
      else
      {
        float boostValue = 1 + 1/pEq->Q * K + W;
        a0 = 1;
        a1 = (2.0f * (W - 1)) / boostValue;
        a2 = ((1 - (K / pEq->Q) + W) / boostValue);
        b0 = (1 + K * gain / pEq->Q + W) / boostValue;
        b1 = 2.0f * (W - 1) / boostValue;
        b2 = (1 - K * gain / pEq->Q + W) / boostValue;
      }
      break;
    }
  case BIQUAD_CALCULATOR_LOW_SHELF:
    {
      float gain = powf(10.0f, pEq->Gain/40.0f);
      alpha = (sinf(omega_c) / 2.0f) * sqrtf((gain + (1.0f / gain)) * (1.0f / pEq->Slope - 1.0f) + 2.0f);
      beta = 2.0f * alpha * sqrtf(gain);
      a0 = (gain + 1.0f) + (gain - 1.0f) * cosf(omega_c) + beta;
      a1 = -2.0 * ((gain - 1.0f) + (gain + 1.0f) * cosf(omega_c));
      a2 = (gain + 1.0f) + (gain - 1.0f) * cosf(omega_c) - beta;
      b0 = gain * ((gain + 1.0f) - (gain - 1.0f) * cosf(omega_c) + beta);
      b1 = 2.0f * gain * ((gain - 1.0f) - (gain + 1.0f) * cosf(omega_c));
      b2 = gain * ((gain + 1.0f) - (gain - 1.0f) * cosf(omega_c) - beta);
      break;
    }
  case BIQUAD_CALCULATOR_HIGH_SHELF:
    {
      float gain = powf(10.0f, pEq->Gain/40.0f);
      alpha = (sinf(omega_c) / 2.0f) * sqrtf((gain + (1.0f / gain)) * (1.0f / pEq->Slope - 1.0f) + 2.0f);
      beta = 2.0f * alpha * sqrtf(gain);
      a0 = (gain + 1.0f) - (gain - 1.0f) * cosf(omega_c) + beta;
      a1 = 2.0f * ((gain - 1.0f) - (gain + 1.0f) * cosf(omega_c));
      a2 = (gain + 1.0f) - (gain - 1.0f) * cosf(omega_c) - beta;
      b0 = gain * ((gain + 1.0f) + (gain - 1.0f) * cosf(omega_c) + beta);
      b1 = -2.0f * gain * ((gain - 1.0f) + (gain + 1.0f) * cosf(omega_c));
      b2 = gain * ((gain + 1.0f) + (gain - 1.0f) * cosf(omega_c) - beta);
      break;
    }
  case BIQUAD_CALCULATOR_NOTCH:
    {
      alpha = sinf(omega_c)/(2*pEq->Q);
      a0 = 1.0f + alpha;
      a1 = -2.0f * cosf(omega_c);
      a2 = 1.0f - alpha;
      b0 = 1.0f;
      b1 = -2.0f * cos(omega_c);
      b2 = 1.0f;
      break;
    }
  case BIQUAD_CALCULATOR_ALL_PASS:
    {
      alpha = sinf(omega_c)/(2*pEq->Q);
      a0 = 1.0f + alpha;
      a1 = -2.0f * cosf(omega_c);
      a2 = 1.0f - alpha;
      b0 = 1.0f - alpha;
      b1 = -2.0f * cos(omega_c);
      b2 = 1.0f + alpha;
      break;
    }
  case BIQUAD_CALCULATOR_BAND_PASS:
    {
      alpha = sinf(omega_c) / (2*pEq->Q);
      float gain = powf(10.0, pEq->Gain / 20.0f);
      a0 = 1.0f + alpha;
      a1 = -2.0f * cosf(omega_c);
      a2 = 1.0f - alpha;
      b0 = alpha * gain;
      b1 = 0;
      b2 = -b0;
      break;
    }
    
  default:
    break;
  }
  
  CoeffSTA350BW[0] = (b1 / 2.0f)/a0;
  CoeffSTA350BW[1] = b2/a0;
  CoeffSTA350BW[2] = (-a1 / 2.0f)/a0;  
  CoeffSTA350BW[3] = -a2/a0;
  CoeffSTA350BW[4] = (b0 / 2.0f)/a0;
  
  float max_value = 0.0f;
  float mult;
  uint16_t i = 0;
  
  for (i = 0; i < K_NUM ; i++)
  {
    if(fabs(CoeffSTA350BW[i]) > max_value)
    {
      max_value = fabs(CoeffSTA350BW[i]);
    }
  }  
  
  if (max_value > 4.0f)
  {
    ret = BIQUAD_CALCULATOR_ERROR;
  }
  else if(max_value > 2.0f)
  {
    ret = BIQUAD_RANGE_FOUR;
  }
  else if(max_value > 1.0f)
  {
    ret = BIQUAD_RANGE_TWO;
  }
  else
  {
    ret = BIQUAD_RANGE_ONE;
  }
  
  mult = powf(2 , 23);
  
  for (i = 0; i < K_NUM ; i++)
  {        
    pEq->Coefficients[i]= (int32_t)((float)CoeffSTA350BW[i] * (float)mult);
  } 
  
  return ret;  
}

/**
* @brief        Some Sound Terminal devices supports coefficients in the range 
*               up to  float [-4 4).
*               When the filter coefficients computed by the BQ_CALC_ComputeFilter function
*               are outside [-1 1) a specific value is returned, the coefficients 
*               must be forced to [-1 1) range 
*               and proper setup on the Sound Terminal device must be performed 
*               in order to treat the values as if they were in the original range. 
*               To force the coefficient in the [-1 1) range a shift is used 
*               inside this function. The amount must be 1 if the values are in the 
*               [-2 2) range or by 2 if in the [-4 4) range. 
* @param        *pEq: pointer to an istance of BIQUAD_Filter_t.
* @param        *coeffRange: range of the coefficients as originally computed.
* @retval       BIQUAD_CALCULATOR_OK if correct operations, BIQUAD_CALCULATOR_ERROR otherwise
*/
int32_t BQ_CALC_ShiftCoefficients(BIQUAD_Filter_t *pEq, uint8_t coeffRange)
{
  if(coeffRange != BIQUAD_RANGE_ONE && coeffRange != BIQUAD_RANGE_TWO && coeffRange != BIQUAD_RANGE_FOUR)
  {
    return BIQUAD_CALCULATOR_ERROR;
  }
  
  uint8_t shift_amount = 0;
  uint8_t i = 0;
  
  if(coeffRange == BIQUAD_RANGE_TWO)
  {
    shift_amount = 1;
  }
  else if(coeffRange == BIQUAD_RANGE_FOUR)
  {
    shift_amount = 2;
  }
  
  if (shift_amount != 0)
  {
    for (i = 0; i < K_NUM; i++)
    {
      pEq->Coefficients[i] >>= shift_amount;
    }
  }
  
  return BIQUAD_CALCULATOR_OK;
  
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
