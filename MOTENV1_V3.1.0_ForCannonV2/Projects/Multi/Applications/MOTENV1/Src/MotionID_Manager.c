/**
 ******************************************************************************
 * @file    MotionID_Manager.c
 * @author  Central LAB
 * @version V3.1.0
 * @date    12-July-2017
 * @brief   This file includes Motion Intensity Detection interface functions
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
#ifdef ENABLE_STM32_MOTION_ID

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MID_output_t MIDCode = MID_ON_DESK; /* on desk */


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionID    Drv_MotionID
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run motion intensity detection algorithm. This function collects and 
*         scale data from accelerometer and calls the motion Intensity Detection Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
void MotionID_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  MID_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;

  MotionID_Update(&iDataIN, &MIDCode);
}

/**
* @brief  Initialises MotionID algorithm
* @param  None
* @retval None
*/
void MotionID_manager_init(void)
{
  char LibVersion[36];
  
  MotionID_Initialize();
  MotionID_GetLibVersion(LibVersion);
  
  TargetBoardFeatures.MotionIDIsInitalized=1;
  MOTENV1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionID        Drv_MotionID*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/
#endif /* ENABLE_STM32_MOTION_ID */

/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
