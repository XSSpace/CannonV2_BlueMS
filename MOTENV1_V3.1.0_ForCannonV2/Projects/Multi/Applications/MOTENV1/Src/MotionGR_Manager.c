/**
 ******************************************************************************
 * @file    MotionGR_Manager.c
 * @author  Central LAB
 * @version V3.1.0
 * @date    12-July-2017
 * @brief   This file includes gesture recognition interface functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "TargetFeatures.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MGR_output_t GestureRecognitionCode = MGR_NOGESTURE;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionGR    Drv_MotionGR
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run gesture recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Gesture Recognition Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
void MotionGR_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  MGR_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;
    
  MotionGR_Update(&iDataIN, &GestureRecognitionCode);
}

/**
* @brief  Initialises MotionGR algorithm
* @param  None
* @retval None
*/
void MotionGR_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];
  
  MotionGR_Initialize();
  MotionGR_GetLibVersion(LibVersion);
  
#ifdef STM32_NUCLEO
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* LSM6DS3/LSM6DSL */
    acc_orientation[0] ='n';
    acc_orientation[1] ='w';
    acc_orientation[2] ='u';
  } else {
    /* LSM6DS0 */
    acc_orientation[0] ='e';
    acc_orientation[1] ='n';
    acc_orientation[2] ='u';
  }
#elif STM32_SENSORTILE
  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';
#endif /* STM32_NUCLEO */

  MotionGR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionGRIsInitalized=1;
  MOTENV1_PRINTF("Initialized %s\n\r", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionGR        Drv_MotionGR*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
