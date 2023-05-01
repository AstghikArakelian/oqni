/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "heartrate11.h"
#include "bmx055.h"
//#include "BMX055.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define ST_IDLE              (uint8_t)0x00
#define ST_START             (uint8_t)0x01
#define ST_STOP		         (uint8_t)0x02

typedef enum
{
  COMMAND_RECEIVED = 0U,
  COMMAND_NOTRECEIVED
} Preambula_StatusTypeDef;

typedef enum
{
  COMMAND_FIRST_BYTE = 0U,
  COMMAND_SECOND_BYTE,
  COMMAND_SWITCH
} Command_ByteCheckTypeDef;

typedef enum
{
  COMMAND_START = 0U,
  COMMAND_STOP,
  COMMAND_NOTR
} Command_StatusTypeDef;

typedef enum
{
  SETUP_DONE = 0U,
  SETUP_NOTDONE
} Setup_StatusTypeDef;

typedef enum
{
  BMX055_OK = 0U,
  BMX055_ERROR
} BMX_StatusTypeDef;

typedef enum
{
  I2C_1 = 0U,
  I2C_2,
  I2C_3
} OB1203X_CheckTypeDef;

typedef enum
{
  I2C1_INT = 1U,
  I2C2_INT,
  I2C3_INT
} OB1203XINT_CheckTypeDef;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
