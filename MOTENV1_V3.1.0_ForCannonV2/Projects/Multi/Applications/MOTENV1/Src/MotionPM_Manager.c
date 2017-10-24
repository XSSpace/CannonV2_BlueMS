/**
 ******************************************************************************
 * @file    MotionPM_Manager.c
 * @author  Central LAB
 * @version V3.1.0
 * @date    12-July-2017
 * @brief   This file includes Pedometer interface functions
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
MPM_output_t PM_DataOUT;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionGR    Drv_MotionPM
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run gesture recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Pedometer Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration values (x/y/z)
* @retval None
*/
void MotionPM_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  MPM_input_t iDataIN;

  iDataIN.AccX = ((float) ACC_Value_Raw.AXIS_X) * sensitivity_Mul;
  iDataIN.AccY = ((float) ACC_Value_Raw.AXIS_Y) * sensitivity_Mul;
  iDataIN.AccZ = ((float) ACC_Value_Raw.AXIS_Z) * sensitivity_Mul;

  MotionPM_Update(&iDataIN, &PM_DataOUT);
}

/**
* @brief  Initialises MotionPM algorithm
* @param  None
* @retval None
*/
void MotionPM_manager_init(void)
{
  char LibVersion[36];
  
  MotionPM_Initialize();
  MotionPM_GetLibVersion(LibVersion);
  
  TargetBoardFeatures.MotionPMIsInitalized=1;
  MOTENV1_PRINTF("Initialized %s\n\r", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionGR        Drv_MotionPM*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
