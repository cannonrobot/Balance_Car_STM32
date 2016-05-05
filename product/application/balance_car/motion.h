#ifndef __MOTION_H
#define __MOTION_H
#include <stdint.h>

typedef struct
{
  int16_t   Steer_Head;       /* Í· */ 
  uint8_t   Steer_Head_Flag;      /*  */
	
  int16_t   Steer_Left_Hand1;    /* ×ó±Û */
  uint8_t   Steer_Left_Hand1_Flag;   /*  */
	
  int16_t   Steer_Right_Hand1; /* ÓÒ±Û*/  
  uint8_t   Steer_Right_Hand1_Flag;   /* */ 
	
  int16_t   Steer_Left_Hand2;   /*×ó±Û2 */   
  uint8_t   Steer_Left_Hand2_Flag;    /*  */
	
  int16_t   Steer_Right_Hand2;      /* ÓÒ±Û2 */
  uint8_t   Steer_Right_Hand2_Flag;    /*  */  

}Steer_Motion_TypeDef;


  void MotionARMLeftThread(void const *argument);
 void MotionARMRightThread(void const *argument);
 void MotionHeadThread(void const *argument);
  void MotionSpeedThread(void const *argument);
   void MotionTurnThread(void const *argument);
void MotionThread(void const *argument);
#endif
