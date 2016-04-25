/**
******************************************************************************
* @file    x_nucleo_cca01m1.c
* @author  Central Labs
* @version V1.0.0
* @date    6-October-2015
* @brief   This file provides a set of functions needed to manage the 
*          X-NUCLEO-CCA01M1 expansion board.
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
#include "x_nucleo_cca01m1.h"
#include "cmsis_os.h"
#ifndef NULL
#define NULL      (void *) 0
#endif


/** @addtogroup BSP
* @{
*/

/** @addtogroup X_NUCLEO_CCA01M1
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL 
* @{
*/

/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL_Private_Variables
* @{
*/ 
static uint32_t I2C_EXPBD_Timeout = NUCLEO_I2C_EXPBD_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
static I2C_HandleTypeDef    I2C_EXPBD_Handle;  

extern   osMutexId osMutexIIC; 
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL_Exported_Function_Prototypes
* @{
*/ 

/* Link function for SOUNDTERMINAL peripheral */
uint8_t            STA350BW_I2C_Init(void);
uint8_t            STA350BW_I2C_ITConfig(void);
uint8_t            STA350BW_I2C_ReadMulti(uint8_t* pBuffer, uint8_t addr, uint8_t reg, uint16_t length);
uint8_t            STA350BW_I2C_WriteMulti(uint8_t* pBuffer, uint8_t addr, uint8_t reg, uint16_t length);
uint8_t            STA350BW_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *value);
uint8_t            STA350BW_I2C_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t            STA350BW_I2C_Delay(uint32_t delay_ms);
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL_Private_Function_Prototypes
* @{
*/ 
static void I2C_EXPBD_MspInit(void);
static void I2C_EXPBD_Error(uint8_t Addr);
static HAL_StatusTypeDef I2C_EXPBD_Init(void);
static HAL_StatusTypeDef I2C_EXPBD_ReadMulti(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
static HAL_StatusTypeDef I2C_EXPBD_WriteMulti(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
static HAL_StatusTypeDef I2C_EXPBD_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t I2C_EXPBD_Read(uint8_t Addr, uint8_t Reg, uint8_t * Value);
/**
* @}
*/

/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL_Exported_Functions 
* @{
*/

/********************************* LINK AUDIOPROCESSOR *****************************/
/**
* @brief  Configures STA350BW I2C interface.
* @param  None
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_Init(void)
{
  if(I2C_EXPBD_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }  
  return HAL_OK;  
}

/**
* @brief  Read multiple bytes using I2C interface.
* @param  *pBuffer: pointer to the array that will contain read data
* @param  addr: device address
* @param  reg: register number 
* @param  length: length of the data to be read 
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_ReadMulti(uint8_t *pBuffer, uint8_t addr, uint8_t reg, uint16_t length)
{ 
  if(I2C_EXPBD_ReadMulti(pBuffer, addr, reg, length) != HAL_OK)
  { 
    return HAL_ERROR;
  }
	
  return HAL_OK;
}

/**
* @brief  Write multiple bytes using I2C interface.
* @param  *pBuffer: pointer to the array that contains data to be written
* @param  addr: device address
* @param  reg: register number 
* @param  length: length of the data to be written 
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_WriteMulti(uint8_t* pBuffer, uint8_t addr, uint8_t reg, uint16_t length)
{  
  if(I2C_EXPBD_WriteMulti(pBuffer, addr, reg, length) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;  
}

/**
* @brief  Read a single byte using I2C interface.
* @param  addr: device address
* @param  reg: register number 
* @param  *value: pointer that will contain read byte 
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_Read(uint8_t addr, uint8_t reg, uint8_t * value)
{
  if(I2C_EXPBD_Read(addr, reg, value) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;    
}

/**
* @brief  Write a single byte using I2C interface.
* @param  addr: device address
* @param  reg: register number 
* @param  value: value to be written 
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_Write(uint8_t addr, uint8_t reg, uint8_t value)
{
  if(I2C_EXPBD_Write(addr, reg, value) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK; 
}

/**
* @brief  Delay function that will be usedinside the componens driver.
* @param  delay_ms: amount of delay, in ms
* @retval HAL_OK 
*/
uint8_t STA350BW_I2C_Delay(uint32_t delay_ms)
{
  HAL_Delay(delay_ms);
  return HAL_OK;
}

/**
* @}
*/


/** @defgroup X_NUCLEO_CCA01M1_LOW_LEVEL_Private_Functions
* @{
*/ 
/******************************* I2C Routines**********************************/
/**
* @brief  Configures I2C interface
* @retval HAL status
*/
static HAL_StatusTypeDef I2C_EXPBD_Init(void)
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  
  if(HAL_I2C_GetState(&I2C_EXPBD_Handle) == HAL_I2C_STATE_RESET)
  {
    /* I2C_EXPBD peripheral configuration */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
    I2C_EXPBD_Handle.Init.ClockSpeed = NUCLEO_I2C_EXPBD_SPEED;
    I2C_EXPBD_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif
    
#if (defined (USE_STM32L0XX_NUCLEO))
    I2C_EXPBD_Handle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;                            /* 400KHz */
#endif
    I2C_EXPBD_Handle.Init.OwnAddress1 = 0x33;
    I2C_EXPBD_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_EXPBD_Handle.Instance = I2C2;    
    I2C_EXPBD_Handle.Init.DualAddressMode =I2C_DUALADDRESS_DISABLED;
    I2C_EXPBD_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
    I2C_EXPBD_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLED;
    /* Init the I2C */
    I2C_EXPBD_MspInit();
    ret_val = HAL_I2C_Init(&I2C_EXPBD_Handle);
  }
  
  return ret_val;
}

/**
* @brief  Write a value in a register of the device through the bus
* @param  *pBuffer: the pointer to data to be written
* @param  Addr: the device address on bus
* @param  Reg: the target register address to be written
* @param  Size: the size in bytes of the value to be written
* @retval HAL status
*/
static HAL_StatusTypeDef I2C_EXPBD_WriteMulti(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
//  if(osMutexWait(osMutexIIC, osWaitForever) == osOK)
  status = HAL_I2C_Mem_Write(&I2C_EXPBD_Handle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout);
 //  osMutexRelease(osMutexIIC);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error(Addr);
  }
  
  return status;
}


/**
* @brief  Read the value of a register of the device through the bus
* @param  *pBuffer: the pointer to data to be read
* @param  Addr: the device address on bus
* @param  Reg: the target register address to be read
* @param  Size: the size in bytes of the value to be read
* @retval HAL status.
*/
static HAL_StatusTypeDef I2C_EXPBD_ReadMulti(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
 // if(osMutexWait(osMutexIIC, osWaitForever) == osOK)
  status = HAL_I2C_Mem_Read(&I2C_EXPBD_Handle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                            I2C_EXPBD_Timeout);
 //    osMutexRelease(osMutexIIC);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error(Addr);
  }
  
  return status;
}

/**
* @brief  Write a value in a register of the device through the bus
* @param  Value: data to be written
* @param  Addr: the device address on bus
* @param  Reg: the target register address to be written
* @retval HAL status
*/
static HAL_StatusTypeDef I2C_EXPBD_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
 // if(osMutexWait(osMutexIIC, osWaitForever) == osOK)
  status = HAL_I2C_Mem_Write(&I2C_EXPBD_Handle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1,
                             I2C_EXPBD_Timeout);
 //    osMutexRelease(osMutexIIC);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error(Addr);
  }
  
  return status;
}


/**
* @brief  Read the value of a register of the device through the bus
* @param  *value: pointer that will contain read byte
* @param  Addr the device address on bus
* @param  Reg the target register address to be read
* @param  Size the size in bytes of the value to be read
* @retval HAL status.
*/
static uint8_t I2C_EXPBD_Read(uint8_t Addr, uint8_t Reg, uint8_t * Value)
{
  HAL_StatusTypeDef status = HAL_OK;  
	//if(osMutexWait(osMutexIIC, osWaitForever) == osOK)
  status = HAL_I2C_Mem_Read(&I2C_EXPBD_Handle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Value, 1,
                            I2C_EXPBD_Timeout);  
//	   osMutexRelease(osMutexIIC);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_EXPBD_Error(Addr);
  }  
  return status;
}

/**
* @brief  Manages error callback by re-initializing I2C
* @param  Addr I2C Address
* @retval None
*/
static void I2C_EXPBD_Error(uint8_t Addr)
{
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit(&I2C_EXPBD_Handle);  
  /*FIXME: We need to wait a while in order to have I2C that works fine after deinit */
  HAL_Delay(1);  
  /* Re-Initiaize the I2C comunication bus */
  I2C_EXPBD_Init();
}

/**
* @brief  I2C MSP Initialization
* @retval None
*/
static void I2C_EXPBD_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable I2C GPIO clocks */
  NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE();
  
  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin =  GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)))
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#endif
  
#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull  = GPIO_PULLUP;//GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = GPIO_AF4_I2C2;
  HAL_GPIO_Init(NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
	
	  GPIO_InitStruct.Pin =  GPIO_PIN_3;	
   GPIO_InitStruct.Alternate  = GPIO_AF9_I2C2;
	   HAL_GPIO_Init(NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  /* Enable the I2C_EXPBD peripheral clock */
 // NUCLEO_I2C_EXPBD_CLK_ENABLE();
  __I2C2_CLK_ENABLE();
  /* Force the I2C peripheral clock reset */
  //NUCLEO_I2C_EXPBD_FORCE_RESET();
  __I2C2_FORCE_RESET();
  /* Release the I2C peripheral clock reset */
 //NUCLEO_I2C_EXPBD_RELEASE_RESET();
  __I2C2_RELEASE_RESET();
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  //HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_EV_IRQn, 0x0A, 0);
  //HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_EV_IRQn);
  
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  //HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_ER_IRQn, 0xA, 0);
  //HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_ER_IRQn);
#endif
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

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
