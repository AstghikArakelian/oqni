/**
  ******************************************************************************
  * @file           : f103.h
  * @brief          : Header for f103.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "Buffer.h"
#include <stdio.h>
#include "string.h"

#ifndef INC_F103_H_
#define INC_F103_H_

#define ob1203_address							0x53

/*RST pin*/
#define ob1203_RST_PIN  	                    GPIO_PIN_4
#define ob1203_RST_GPIO_PORT               		GPIOB


/*CLK pin*/
#define ob1203_CLK_PIN                     	    GPIO_PIN_5
#define ob1203_CLK_GPIO_PORT               	 	GPIOA
//#define ob1203_CLK_AF                      		GPIO_AF5_SPI1


/*RDY interrupt pin for I2C1*/
#define ob1203_GP1INT_PIN                  	 	GPIO_PIN_3
#define ob1203_GP1INT_GPIO_PORT            	 	GPIOB
#define ob1203_GP1INT_IRQn                 	 	EXTI3_IRQn


#define ob1203_GP2INT_PIN                       GPIO_PIN_5
#define ob1203_GP2INT_GPIO_PORT                 GPIOB
#define ob1203_GP2INT_IRQn                      EXTI9_5_IRQn


#define I2C_TIMEOUT_FLAG          35U         /*!< Timeout 35 ms             */
#define I2C_TIMEOUT_BUSY_FLAG     25U         /*!< Timeout 25 ms             */
#define I2C_TIMEOUT_STOP_FLAG     5U          /*!< Timeout 5 ms              */
#define I2C_NO_OPTION_FRAME       0xFFFF0000U /*!< XferOptions default value */

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((uint32_t)((uint32_t)HAL_I2C_STATE_BUSY_TX | (uint32_t)HAL_I2C_STATE_BUSY_RX) & (uint32_t)(~((uint32_t)HAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE))                                                        /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */


void ob1203_RstSet(void);
void ob1203_RstReset(void);
HAL_StatusTypeDef ob1203_I2C_Write(uint8_t * data, uint8_t count);
HAL_StatusTypeDef ob1203_I2C_Read(uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count);
void ob1203_Delay_ms(uint32_t ms);
void ob1203_send_results(uint32_t ppg);
void ob1203_send_noresult(void);


#endif /* INC_F103_H_ */
