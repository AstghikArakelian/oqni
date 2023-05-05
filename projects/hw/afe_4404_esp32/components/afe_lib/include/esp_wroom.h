/**
  ******************************************************************************
  * @file           : f103.h
  * @brief          : Header for f103.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "../../buffer/include/buffer.h"
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"



#ifndef INC_F103_H_
#define INC_F103_H_

#define afe4404_address							0x58

//extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart1;
/*I2C1*/
#define afe4404_I2C1_SCL_PIN                     19
#define afe4404_I2C1_SDA_PIN                   	 18

/*I2C2*/
#define afe4404_I2C2_SCL_PIN                     22
#define afe4404_I2C2_SDA_PIN                     21


/*RST pin*/
#define afe4404_RST_PIN  	                     33


/*CLK pin*/
//#define afe4404_CLK_PIN                     	 GPIO_PIN_5


/*RDY interrupt pin for I2C1*/
#define afe4404_GP1INT_PIN                  	 34

#define afe4404_GP2INT_PIN                       35


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


void afe4404_RstSet(void);
void afe4404_RstReset(void);
void afe4404_I2C_Write(uint8_t * data, uint8_t count);
void afe4404_I2C_Read(uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count);
void afe4404_Delay_ms(uint32_t ms);
void afe4404_send_results(uint8_t num, uint32_t led1, uint32_t led2, uint32_t led3);
void afe4404_send_noresult(void);
void afe4404_send_preambula(void);

#endif /* INC_F103_H_ */
