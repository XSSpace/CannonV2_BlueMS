/**
  ******************************************************************************
  * @file    main.h 
  * @author  Central LAB
  * @version V3.1.0
  * @date    12-July-2017
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "console.h" 

#include "osal.h"
#include "debug.h"
#include "MOTENV1_config.h"

#ifndef USE_STM32L0XX_NUCLEO
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
#endif /* USE_STM32L0XX_NUCLEO */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

#ifndef USE_STM32L0XX_NUCLEO
extern unsigned char ReCallCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data);
#endif /* USE_STM32L0XX_NUCLEO */

/* Exported defines and variables  ------------------------------------------------------- */


#ifndef USE_STM32L0XX_NUCLEO
  //10kHz/100 For MotionFX or MotionGR@100Hz
  #define DEFAULT_uhCCR1_Val  100
#else /* USE_STM32L0XX_NUCLEO */
  //10kHz/2 For Environmental data@2Hz
  #define DEFAULT_uhCCR1_Val  5000
#endif /* USE_STM32L0XX_NUCLEO */

//10kHz/50 For MotionCP or MotionPM@50Hz
#define DEFAULT_uhCCR2_Val  200
//10kHz/16  For MotionAR@16Hz
#define DEFAULT_uhCCR3_Val  625

//10kHz/20  For Acc/Gyromag@20Hz
#define DEFAULT_uhCCR4_Val  500

#ifndef STM32_NUCLEO
  #define BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER ((uint32_t)0x12345678)
#endif /* STM32_NUCLEO */

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
