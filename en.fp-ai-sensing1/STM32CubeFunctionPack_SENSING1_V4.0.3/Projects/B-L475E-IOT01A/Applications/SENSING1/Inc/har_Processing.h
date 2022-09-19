/**
  ******************************************************************************
  * @file    har_Processing.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    30-Oct-2019
  * @brief   API defined on har_Processing.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#ifndef _HAR_PROCESSING_H_
#define _HAR_PROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "SENSING1.h"
#include "ai_platform.h"
#include "ai_common.h"

/* Exported define ------------------------------------------------------------*/

#define HAR_IN_MAX_SIZE  AI_HAR_GMP_IN_1_SIZE

#if     AI_HAR_IGN_IN_1_SIZE > HAR_IN_MAX_SIZE
#undef  HAR_IN_MAX_SIZE
#define HAR_IN_MAX_SIZE AI_HAR_IGN_IN_1_SIZE
#endif

#if     AI_HAR_IGN_WSDM_IN_1_SIZE > HAR_IN_MAX_SIZE
#undef  HAR_IN_MAX_SIZE
#define HAR_IN_MAX_SIZE AI_HAR_IGN_WSDM_IN_1_SIZE
#endif

#define HAR_OUT_MAX_SIZE  AI_HAR_GMP_OUT_1_SIZE

#if     AI_HAR_IGN_OUT_1_SIZE > HAR_OUT_MAX_SIZE
#undef  HAR_OUT_MAX_SIZE
#define HAR_OUT_MAX_SIZE AI_HAR_IGN_OUT_1_SIZE
#endif

#if     AI_HAR_IGN_WSDM_OUT_1_SIZE > HAR_OUT_MAX_SIZE
#undef  HAR_OUT_MAX_SIZE
#define HAR_OUT_MAX_SIZE AI_HAR_IGN_WSDM_OUT_1_SIZE
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    HAR_ALGO_IDX_NONE           = 0xFF,
    HAR_GMP_IDX                 = 0x00,
    HAR_IGN_IDX                 = 0x01,
    HAR_IGN_WSDM_IDX            = 0x02,
    HAR_ALGO_IDX_NUMBER         = 0x03
} HAR_algoIdx_t;

 typedef struct
{
  float AccX;           /*  acc x axes [g]  */
  float AccY;           /*  acc y axes [g]  */
  float AccZ;           /*  acc z axes [g]  */
} HAR_input_t;

typedef enum
{
    HAR_NOACTIVITY          = 0x00,
    HAR_STATIONARY          = 0x01,
    HAR_WALKING             = 0x02,
    HAR_FASTWALKING         = 0x03,
    HAR_JOGGING             = 0x04,
    HAR_BIKING              = 0x05,
    HAR_DRIVING             = 0x06,
    HAR_STAIRS              = 0x07
} HAR_output_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup NN_AR_Exported_Functions NN_AR_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */


/**
 * @brief  Initialize the Human Activity Recognition engine
 * @param  None
 * @retval 0 in case of success, a negative code otherwise
 */
int8_t  HAR_Initialize(HAR_algoIdx_t algo);

/**
 * @brief  deInitialize the MotionAR engine
 * @param  None
 * @retval None
 */
int8_t HAR_DeInitialize(HAR_algoIdx_t algo);

/**
 * @brief  Run Activity Recognition Algorithm
 * @param  data_in: pointer to the HAR_input_t structure
 * @retval activity index
 */
HAR_output_t HAR_run(MOTION_SENSOR_AxesRaw_t ACC_Value_Raw, HAR_algoIdx_t algo);

/**
 * @brief  get latest activity code computes by Recognition Algorithm
 * @param  None
 * @retval activity index
 */
HAR_output_t HAR_get_Activity_Code(HAR_algoIdx_t algo);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
// uint8_t HAR_GetLibVersion(char *version);


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* HAR_PROCESSING */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
