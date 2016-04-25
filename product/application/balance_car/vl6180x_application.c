/**
 ******************************************************************************
 * File Name          : multi_dev.c
 * Date               : 12/1/2015 
 * Description        : multi bear program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
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

#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c1;

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "vl6180x_api.h"
#include "x-nucleo-6180xa1.h"

#ifdef DEBUG
#include "diag/trace.h"
#define debug(msg, ...)   trace_printf(msg,__VA_ARGS__)
#define trace_warn(msg,...) trace_printf("W %s %d" msg "\n", __func__, __LINE__, __VA_ARGS__)
#else
#define debug(msg, ...)  (void)0
#endif

#define DigitDisplay_ms     1 /* ms each digit is kept on */

#define PressBPSwicthTime   1000 /* if user keep bp press more that this mean swicth mode else rooll over use c&se in mode */

void WaitMilliSec(int ms);

VL6180xDev_t theVL6180xDev;
struct MyVL6180Dev_t BoardDevs[4] = { [0]= { .DevID = 0 }, [1]= { .DevID = 1 }, [2]= { .DevID = 2 }, [3]= { .DevID = 3 }, };
VL6180xDev_t theVL6180xDev = &BoardDevs[0];

/**
 * VL6180x CubeMX F401 multiple device i2c implementation
 */

#define i2c_bus      (&hi2c1)
#define def_i2c_time_out 100

int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Transmit(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status? -1 : 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Receive(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }

    return status? -1 : 0;
}

/**
 * platform and application specific for XNUCLEO6180XA1 Expansion Board
 */
void XNUCLEO6180XA1_WaitMilliSec(int n) {
    WaitMilliSec(n);
}

volatile int IntrFired = 0;

void XNUCLEO6180XA1_UserIntHandler(void) {
    IntrFired++;
}

/**
 * DISPLAY public
 */
/***************  DISPLAY PUBLIC *********************/
const char *DISP_NextString;
/***************  DISPLAY PRIVATE *********************/
static char DISP_CurString[10];
static int DISP_Loop = 0;

void DISP_ExecLoopBody(void) {
    if (DISP_NextString != NULL) {
        strncpy(DISP_CurString, DISP_NextString, sizeof(DISP_CurString) - 1);
        DISP_CurString[sizeof(DISP_CurString) - 1] = 0;
        DISP_NextString = NULL;
    }
    XNUCLEO6180XA1_DisplayString(DISP_CurString, DigitDisplay_ms);
    DISP_Loop++;
}

/**
 * Nucleo board specific
 *
 */

#define BSP_BP_PORT GPIOC
#define BSP_BP_PIN  GPIO_PIN_13
int BSP_GetPushButton(void) {
    GPIO_PinState state;
    state = HAL_GPIO_ReadPin(BSP_BP_PORT, BSP_BP_PIN);
    return state;
}

void BSP_Init(){
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /*Configure push button GPIO pin : PC13  */
    __GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SetDisplayString(const char *msg) {
    DISP_NextString = msg;
}

void WaitMilliSec(int ms) {
    HAL_Delay(ms); /* it's milli sec  cos we do set systick to 1KHz */
}

/**
 * call in the main loop
 * when running under debugger it enable doing direct vl6180x reg access
 * After  breaking at entrance
 * change  the the local index/data and cmd variable to do what needed
 * reg_cmd -1 wr byte -2wr word -3 wr dword
 * reg_cmd 1 rd byte 2 rd word 3 rd dword
 * step to last statement before return and read variable to get rd result exit
 */
void debug_stuff(void) {
    int reg_cmd = 0;
    static uint32_t reg_data;
    static uint16_t reg_index;

    if (reg_cmd) {
        switch (reg_cmd) {
            case -1:
                VL6180x_WrByte(theVL6180xDev, reg_index, reg_data);
                debug("Wr B 0x%X = %d", reg_index, (int )reg_data);
                break;
            case -2:
                VL6180x_WrWord(theVL6180xDev, reg_index, reg_data);
                debug("Wr W 0x%X = %d", reg_index, (int ) reg_data);
                break;

            case -3:
                VL6180x_WrDWord(theVL6180xDev, reg_index, reg_data);
                debug("WrDW 0x%X = %d", reg_index, (int )reg_data);
                break;

            case 1:
                reg_data = 0;
                VL6180x_RdByte(theVL6180xDev, reg_index, (uint8_t*) &reg_data);
                debug("RD B 0x%X = %d", reg_index, (int )reg_data);
                break;
            case 2:
                reg_data = 0;
                VL6180x_RdWord(theVL6180xDev, reg_index, (uint16_t*) &reg_data);
                debug("RD W 0x%X = %d", reg_index, (int )reg_data);
                break;

            case 3:
                VL6180x_RdDWord(theVL6180xDev, reg_index, &reg_data);
                debug("RD DW 0x%X = %d", reg_index, (int )reg_data);
                break;
            default:
                debug("Invalid command %d", reg_cmd);
                /* nothing to do*/
                ;
        }
    }
}

/**
 * When button is already pressed it Wait for user to release it
 * if button remain pressed for given time it return true
 * These is used to detect mode switch by long press on blue Push Button
 *
 * As soon as time is elapsed -rb- is displayed  to let user know order
 * the  request to switch mode is taken into account
 *
 * @return True if button remain pressed more than specified time
 */
int PusbButton_WaitUnPress(void) {
    uint32_t TimeStarted = HAL_GetTick();
    uint32_t tick;

    while (!BSP_GetPushButton()) {
        ; /* debounce */
        DISP_ExecLoopBody();
        tick = HAL_GetTick();
        if (-TimeStarted > PressBPSwicthTime) {
            SetDisplayString(" rb ");
        }
    }
    return tick - TimeStarted > PressBPSwicthTime;
}

void DisplayScroll_UserWait( const char * msg)
{
    int char_time=250;
    int len;
    int offset=0;
    int last_wait=0;

    uint32_t started;
    len=strlen(msg);

    started=HAL_GetTick();
    while( BSP_GetPushButton()  ){
        SetDisplayString(msg+offset);
        DISP_ExecLoopBody();
        if(HAL_GetTick()-started > char_time ){
            started=HAL_GetTick();
            if( offset+4>=len ){
                if( last_wait++ < 4 ){
                    continue;
                }
                last_wait=0;
                offset=0;
            }
            else{
                offset++;
            }
        }
    }
    while( !BSP_GetPushButton() ){
        DISP_ExecLoopBody();
    }
}


void HandleError(const char *msg) {
    static char ErrString[256]="Err ";
    strcat(ErrString,msg);
    while (1) {
        DisplayScroll_UserWait(ErrString);
    }
}

void SystemClock_Config(void);
    
void DisplayDelay(int ms){
    uint32_t started;
    
    started= HAL_GetTick();    
    while(HAL_GetTick()-started < ms ){
          DISP_ExecLoopBody();
    }
}


 
void DisplayUntilButton(const char *msg){
    SetDisplayString(msg);
    DISP_ExecLoopBody();
    /* wait for button to be pressed first */
    while( BSP_GetPushButton()) {
        DISP_ExecLoopBody();
    }
    PusbButton_WaitUnPress();
}

 #define MAX_DEV 4
    int status;
    int i;
    int n_dev;
    int PresentDevMask;
    int nPresentDevs;
    int PresentDevIds[MAX_DEV];
    VL6180x_RangeData_t Range[MAX_DEV];
    int nReady;
    char DisplayStr[5];

void v16180x_Init(void){
	
    /* MCU Configuration----------------------------------------------------------*/


    /* initialize VL6180x expansion board gpio and i2c */
   // XNUCLEO6180XA1_GPIO_Init();
   // XNUCLEO6180XA1_I2C1_Init(&hi2c1);

    if (!XNUCLEO6180XA1_IsV2()) {
        HandleError("V2 expansion board is required");
        n_dev=1;
    } else {
        n_dev = 4;
    }

    /* put all devices under reset */
    for (i = 0; i < n_dev; i++) {
        /* put all under reset */
        XNUCLEO6180XA1_ResetId(0, i);
    }
    

    /* detect presence and initialize devices i2c address  */
    /*set device i2c address for dev[i] = 0x52+(i+1)*2 */
    PresentDevMask = 0;
    nPresentDevs = 0;
    strcpy(DisplayStr,"TLBR");
    for (i = n_dev - 1; i >= 0; i--) {
        int FinalI2cAddr;
        uint8_t id;
        
        /* unreset device that wake up at default i2c addres 0x52 */
        XNUCLEO6180XA1_ResetId(1, i);
        WaitMilliSec(2);    /* at least 400usec before to acces device */
        BoardDevs[i].I2cAddr = 0x52;
        /* to detect device presence try to read it's dev id */
        status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
        if (status) {
            /* these device is not present skip init and clear it's letter on string */
            BoardDevs[i].Present = 0;
            DisplayStr[i]=' ';
            continue;
        }
        /* device present only */
        BoardDevs[i].Present = 1;
        PresentDevMask |= 1 << i;
        PresentDevIds[nPresentDevs]=i;
        nPresentDevs++;
        status = VL6180x_InitData(&BoardDevs[i]);

        FinalI2cAddr = 0x52 + ((i+1) * 2);
        if (FinalI2cAddr != 0x52) {
            status = VL6180x_SetI2CAddress(&BoardDevs[i], FinalI2cAddr);
            if( status ){
                HandleError("VL6180x_SetI2CAddress fail");
            }
            BoardDevs[i].I2cAddr = FinalI2cAddr;
        }

        WaitMilliSec(1);
        status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
        WaitMilliSec(1);
        status= VL6180x_Prepare(&BoardDevs[i]);
        if( status<0 ){
            HandleError("VL6180x_Prepare fail");
        }
        /* Disable Dmax computation */
        VL6180x_DMaxSetState(&BoardDevs[i], 0);
    }
	
	
	
}

int vl6180x_Get_Value(void){
	
	 // DisplayStr[4]=0;
   // DisplayScroll_UserWait(DisplayStr);
char buffer[10];
    while (1) {
        VL6180xDev_t dev;
        dev =  BoardDevs + PresentDevIds[0]+PresentDevIds[1];
	VL6180x_RangePollMeasurement(theVL6180xDev, &Range[1]);
			
			
			/*
			
            dev =  BoardDevs + PresentDevIds[0]+PresentDevIds[1];
            status = VL6180x_RangeStartSingleShot(dev);
           
            dev->Ready=0;
        
    
        nReady=0;
        do{
            DISP_ExecLoopBody();
          
                dev =  BoardDevs + PresentDevIds[0]+PresentDevIds[1];
                if( !dev->Ready ){
                    status = VL6180x_RangeGetMeasurementIfReady(dev, &Range[1]);
                    if( status == 0 ){
                        if(Range[1].errorStatus == DataNotReady)
                            continue;
                      
                        dev->Ready=1;
												  nReady++;
                    } else {
                        HandleError("VL6180x_RangeStartSingleShot fail");
                    }
                }
					 }
          while( nReady<1);    
       
				*/ 
        /* use all measure now */
        /* build the display string  each */
        strcpy(DisplayStr,"----");
      
            if( Range[1].errorStatus == 0 ){
                int ch;
							 sprintf(buffer, "r%3d", (int) Range[1].range_mm);
							 // SetDisplayString( buffer );
							/*
                if( Range[1].range_mm < 60 ){
                    ch= '_';
                }
                else if( Range[1].range_mm < 120){
                    ch= '=';
                }
                else{
                    ch= '~';
                }
                DisplayStr[BoardDevs[PresentDevIds[1]].DevID]=ch;
							*/
            }
            else{
                /* no distance shut off segment */
              //  DisplayStr[BoardDevs[PresentDevIds[1]].DevID]=' ';
							 sprintf(buffer, "r    " );
           }
        
        SetDisplayString(buffer);

    }
	
	
}


int vl6180x_application(void) {
   
   // DisplayStr[4]=0;
   // DisplayScroll_UserWait(DisplayStr);

    while (1) {
        VL6180xDev_t dev;
        /* kick off measure on all device */
        for( i=0; i<nPresentDevs; i++){
            dev =  BoardDevs + PresentDevIds[i];
            status = VL6180x_RangeStartSingleShot(dev);
            if( status<0 ){
                HandleError("VL6180x_RangeStartSingleShot fail");
            }
            dev->Ready=0;
        }
        /* wait for all present device to have a measure */ 
        nReady=0;
        do{
            DISP_ExecLoopBody();
            for( i=0; i<nPresentDevs; i++){
                dev =  BoardDevs + PresentDevIds[i];
                if( !dev->Ready ){
                    status = VL6180x_RangeGetMeasurementIfReady(dev, &Range[i]);
                    if( status == 0 ){
                        if(Range[i].errorStatus == DataNotReady)
                            continue;
                        /* New measurement ready */
                        dev->Ready=1;
                        nReady++;
                    } else {
                        HandleError("VL6180x_RangeStartSingleShot fail");
                    }
                }
            }
        }
        while( nReady<nPresentDevs);

        /* use all measure now */
        /* build the display string  each */
        strcpy(DisplayStr,"----");
        for( i=0; i<nPresentDevs; i++){
            if( Range[i].errorStatus == 0 ){
                int ch;
                if( Range[i].range_mm < 60 ){
                    ch= '_';
                }
                else if( Range[i].range_mm < 120){
                    ch= '=';
                }
                else{
                    ch= '~';
                }
                DisplayStr[BoardDevs[PresentDevIds[i]].DevID]=ch;
            }
            else{
                /* no distance shut off segment */
                DisplayStr[BoardDevs[PresentDevIds[i]].DevID]=' ';
            }
        }
        SetDisplayString(DisplayStr);

    }

}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    trace_printf("assert_param() failed: file \"%s\", line %d\n", file, line);
    exit(1);
    /* USER CODE END 6 */

}

#endif

