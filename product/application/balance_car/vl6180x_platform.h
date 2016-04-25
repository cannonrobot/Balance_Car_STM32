/*******************************************************************************
################################################################################
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License version 2 and only version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
********************************************************************************/


/* vl6180x_platform.h STM32 Nucleo F401 single device  sample code project */

#ifndef VL6180x_PLATFORM
#define VL6180x_PLATFORM


#include "vl6180x_def.h"


#define VL6180x_SINGLE_DEVICE_DRIVER 	0
#define VL6180x_RANGE_STATUS_ERRSTRING  1
#define VL6180X_SAFE_POLLING_ENTER 		0
#define VL6180X_LOG_ENABLE			    0


#define VL6180x_DEV_DATA_ATTR
#define ROMABLE_DATA


#if VL6180X_LOG_ENABLE
/*  dot not include non ansi here trace was a case :( */
#ifdef TRACE
#include "diag/trace.h"
#define LOG_GET_TIME()  HAL_GetTick()
#else
/* these is nto stm32 vl6180x GNuArm eclpse build*/
#define trace_printf(...) (void)0
#define LOG_GET_TIME() (int)0 /* add your code here expect to be an integer native (%d) type  value  */
#endif



#define LOG_FUNCTION_START(fmt, ... ) \
    trace_printf("beg %s start @%d\t" fmt "\n", __func__, LOG_GET_TIME(), ##__VA_ARGS__)

#define LOG_FUNCTION_END(status)\
        trace_printf("end %s @%d %d\n", __func__, LOG_GET_TIME(), (int)status)

#define LOG_FUNCTION_END_FMT(status, fmt, ... )\
        trace_printf("End %s @%d %d\t"fmt"\n" , __func__, LOG_GET_TIME(), (int)status, ##__VA_ARGS__)

#define VL6180x_ErrLog(msg, ... )\
    do{\
        trace_printf("ERR in %s line %d\n" msg, __func__, __LINE__, ##__VA_ARGS__);\
    }while(0)

#else /* VL6180X_LOG_ENABLE no logging */

	#define LOG_FUNCTION_START(...) (void)0
	#define LOG_FUNCTION_END(...) (void)0
	#define LOG_FUNCTION_END_FMT(...) (void)0
    #define VL6180x_ErrLog(... ) (void)0
#endif


#if  VL6180x_SINGLE_DEVICE_DRIVER
    #error "VL6180x_SINGLE_DEVICE_DRIVER must be set"
#endif
		
struct MyVL6180Dev_t {
		struct VL6180xDevData_t Data;
		uint8_t I2cAddr;
		uint8_t DevID;
    
	// misc flags for application 	
    unsigned Present:1;
    unsigned Ready:1;
};
typedef struct MyVL6180Dev_t *VL6180xDev_t;

#define VL6180xDevDataGet(dev, field) (dev->Data.field)
#define VL6180xDevDataSet(dev, field, data) (dev->Data.field)=(data)


void VL6180x_PollDelay(VL6180xDev_t dev);

void DISP_ExecLoopBody(void);
#define VL6180x_PollDelay(dev)   DISP_ExecLoopBody();


#endif  /* VL6180x_PLATFORM */



