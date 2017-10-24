/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  Central LAB
  * @version V3.1.0
  * @date    12-July-2017
  * @brief   Specification of the HW Features for each target platform
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
  
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
   
#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_hal.h"
  #ifdef STM32_NUCLEO
    #include "stm32f4xx_nucleo.h"
    #include "stm32f4xx_nucleo_bluenrg.h"
    #include "stm32f4xx_hal_conf.h"
    #include "stm32f4xx_UART.h"
    #include "stm32f4xx_I2C.h"
    #include "stm32f4xx_SPI.h"
    #include "stm32f4xx_periph_conf.h"
    #include "stm32_bluenrg_ble.h"
   
    #include "x_nucleo_iks01a2.h"
    #include "x_nucleo_iks01a2_accelero.h"
    #include "x_nucleo_iks01a2_gyro.h"
    #include "x_nucleo_iks01a2_magneto.h"
    #include "x_nucleo_iks01a2_humidity.h"
    #include "x_nucleo_iks01a2_temperature.h"
    #include "x_nucleo_iks01a2_pressure.h"

    #include "x_nucleo_iks01a1.h"
    #include "x_nucleo_iks01a1_accelero.h"
    #include "x_nucleo_iks01a1_gyro.h"
    #include "x_nucleo_iks01a1_magneto.h"
    #include "x_nucleo_iks01a1_humidity.h"
    #include "x_nucleo_iks01a1_temperature.h"
    #include "x_nucleo_iks01a1_pressure.h"
  #endif /* STM32_NUCLEO */
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_hal.h"
  #ifdef STM32_NUCLEO
    #include "stm32l4xx_nucleo.h"
    #include "stm32l4xx_nucleo_bluenrg.h"
    #include "stm32l4xx_hal_conf.h"
    #include "stm32l4xx_UART.h"
    #include "stm32l4xx_I2C.h"
    #include "stm32l4xx_SPI.h"
    #include "stm32l4xx_periph_conf.h"
    #include "stm32_bluenrg_ble.h"

      #include "x_nucleo_iks01a2.h"
      #include "x_nucleo_iks01a2_accelero.h"
      #include "x_nucleo_iks01a2_gyro.h"
      #include "x_nucleo_iks01a2_magneto.h"
      #include "x_nucleo_iks01a2_humidity.h"
      #include "x_nucleo_iks01a2_temperature.h"
      #include "x_nucleo_iks01a2_pressure.h"
   
      #include "x_nucleo_iks01a1.h"
      #include "x_nucleo_iks01a1_accelero.h"
      #include "x_nucleo_iks01a1_gyro.h"
      #include "x_nucleo_iks01a1_magneto.h"
      #include "x_nucleo_iks01a1_humidity.h"
      #include "x_nucleo_iks01a1_temperature.h"
      #include "x_nucleo_iks01a1_pressure.h"
  #elif STM32_SENSORTILE
    #include "SensorTile.h"
    #include "stm32l4xx_hal_conf.h"
    #include "stm32l4xx_hal_def.h"
    #include "SensorTile_BlueNRG.h"
    #include "SensorTile_accelero.h"
    #include "SensorTile_gyro.h"
    #include "SensorTile_magneto.h"
    #include "SensorTile_pressure.h"
    #include "SensorTile_temperature.h"
    #include "SensorTile_humidity.h"
    #include "SensorTile_gg.h"
    #include "SensorMappFunc.h"
#endif /* STM32_NUCLEO */
#endif /* USE_STM32L4XX_NUCLEO */
   
#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_hal.h"
  #ifdef STM32_NUCLEO
    #include "stm32l0xx_nucleo.h"
    #include "stm32l0xx_nucleo_bluenrg.h"
    #include "stm32l0xx_hal_conf.h"
    #include "stm32l0xx_I2C.h"
    #include "stm32l0xx_SPI.h"
    #include "stm32l0xx_periph_conf.h"
    #include "stm32_bluenrg_ble.h"
     #ifdef IKS01A1
      #include "x_nucleo_iks01a1.h"
      #include "x_nucleo_iks01a1_accelero.h"
      #include "x_nucleo_iks01a1_gyro.h"
      #include "x_nucleo_iks01a1_magneto.h"
      #include "x_nucleo_iks01a1_humidity.h"
      #include "x_nucleo_iks01a1_temperature.h"
      #include "x_nucleo_iks01a1_pressure.h"
    #elif IKS01A2
      #include "x_nucleo_iks01a2.h"
      #include "x_nucleo_iks01a2_accelero.h"
      #include "x_nucleo_iks01a2_gyro.h"
      #include "x_nucleo_iks01a2_magneto.h"
      #include "x_nucleo_iks01a2_humidity.h"
      #include "x_nucleo_iks01a2_temperature.h"
      #include "x_nucleo_iks01a2_pressure.h"
    #endif /* IKS01A1 */
      #include "SensorMappFunc.h"
  #endif /* STM32_NUCLEO */
#endif /* USE_STM32L0XX_NUCLEO */
   
#include "MOTENV1_config.h"
   
#ifndef USE_STM32L0XX_NUCLEO
#include "MetaDataManager.h"

#include "MotionFX_Manager.h"
#include "motion_fx.h"

#include "MotionCP_Manager.h"
#include "motion_cp.h"

#include "MotionAR_Manager.h"
#include "motion_ar.h"

#include "MotionGR_Manager.h"
#include "motion_gr.h"

#include "MotionPM_Manager.h"
#include "motion_pm.h"

#ifdef ENABLE_STM32_MOTION_ID
  #include "MotionID_Manager.h"
  #include "motion_id.h"
#endif /* ENABLE_STM32_MOTION_ID */
   
#endif /* USE_STM32L0XX_NUCLEO */

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2

/* BlueNRG Board Type */
#define IDB04A1 0
#define IDB05A1 1

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */
/**
 * @brief  Target type data definition
 */
typedef enum
{
  TARGET_NUCLEO,
  TARGET_BLUECOIN,
  TARGET_SENSORTILE,
  TARGETS_NUMBER
} TargetType_t;

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  TargetType_t BoardType;
  int32_t NumTempSensors;
  void *HandleTempSensors[MAX_TEMP_SENSORS];

  void *HandlePressSensor;
  void *HandleHumSensor;

  int32_t HWAdvanceFeatures;
  void *HandleAccSensor;
  void *HandleGyroSensor;
  void *HandleMagSensor;

  uint8_t LedStatus;
  uint8_t bnrg_expansion_board;
  uint8_t SnsAltFunc;

#ifdef STM32_SENSORTILE
  void *HandleGGComponent;
#endif /* STM32_SENSORTILE */

  /* Sensor Fusion Library */
  uint32_t MotionFXIsInitalized;

  /* Carry Position Library */
  uint32_t MotionCPIsInitalized;

  /* Activity Recognition Library */
  uint32_t MotionARIsInitalized;

  /* Gesture Recognition Library */
  uint32_t MotionGRIsInitalized;

  /* Pedometer Library */
  uint32_t MotionPMIsInitalized;

#ifdef ENABLE_STM32_MOTION_ID
  /* Motion Intensity Detection Library */
  uint32_t MotionIDIsInitalized;
#endif /* ENABLE_STM32_MOTION_ID */
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(TargetType_t BoardType);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

