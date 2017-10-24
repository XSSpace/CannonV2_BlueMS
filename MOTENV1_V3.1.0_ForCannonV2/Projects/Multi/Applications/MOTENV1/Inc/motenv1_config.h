/**
  ******************************************************************************
  * @file    MOTENV1_config.h
  * @author  Central LAB
  * @version V3.1.0
  * @date    12-July-2017
  * @brief   FP-SNS-MOTENV1 configuration
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
#ifndef __MOTENV1_CONFIG_H
#define __MOTENV1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* Define the MOTENV1 MAC address, otherwise it will create a MAC */
//#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#ifndef MAC_BLUEMS
  /* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
  #define MAC_STM32UID_BLUEMS
#endif /* MAC_BLUEMS */

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* For Enabling or not the MotionID. Not yet Supported by BlueMS application */
#define ENABLE_STM32_MOTION_ID

/* IMPORTANT
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#ifdef STM32_SENSORTILE
  /* Enabling this define for SensorTile..
   * it will introduce a delay of 10Seconds before starting the application
   * for having time to open the Terminal
   * for looking the MOTENV1 Initialization phase */
  //#define MOTENV1_ENABLE_PRINTF
#else /* STM32_SENSORTILE */
  #ifndef USE_STM32L0XX_NUCLEO
    /* For Nucleo F401RE/L476RG it's enable by default */
    #define MOTENV1_ENABLE_PRINTF
  #endif /* USE_STM32L0XX_NUCLEO */
#endif /* STM32_SENSORTILE */

/* For enabling connection and notification subscriptions debug */
#define MOTENV1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define MOTENV1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define MOTENV1_VERSION_MAJOR '3'
#define MOTENV1_VERSION_MINOR '1'
#define MOTENV1_VERSION_PATCH '0'

/* Define the MOTENV1 Name MUST be 7 char long */
#define NAME_BLUEMS 'M','E','1','V',MOTENV1_VERSION_MAJOR,MOTENV1_VERSION_MINOR,MOTENV1_VERSION_PATCH

/* Package Name */
#define MOTENV1_PACKAGENAME "FP-SNS-MOTENV1"

#ifdef MOTENV1_ENABLE_PRINTF
  #ifdef STM32_NUCLEO
    #define MOTENV1_PRINTF(...) printf(__VA_ARGS__)
  #elif STM32_SENSORTILE
    #include "usbd_cdc_interface.h"
    #define MOTENV1_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
  #endif /* STM32_NUCLEO */
#else /* MOTENV1_ENABLE_PRINTF */
  #define MOTENV1_PRINTF(...)
#endif /* MOTENV1_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FF80050)
#endif /* USE_STM32L0XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __MOTENV1_CONFIG_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
