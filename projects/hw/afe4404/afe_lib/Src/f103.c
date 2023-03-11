/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : f103.c
  * @brief          : Functions for communicating with afe4404
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "f103.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/
extern uint8_t transmit_cplt;
extern I2C_HandleTypeDef cur_i2c;

/* Private user code ---------------------------------------------------------*/
void afe4404_RstSet(void)
{
	HAL_GPIO_WritePin(afe4404_RST_GPIO_PORT, afe4404_RST_PIN, SET);
}


void afe4404_RstReset(void)
{
	HAL_GPIO_WritePin(afe4404_RST_GPIO_PORT, afe4404_RST_PIN, RESET);
}

void afe4404_I2C_Write(uint8_t * data, uint8_t count)
{
	HAL_I2C_Master_Transmit(&cur_i2c, afe4404_address<< 1, data, count, 10);
}

void afe4404_I2C_Read(uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count)
{
	HAL_I2C_Mem_Read(&cur_i2c, afe4404_address<< 1, *reg, cmd_size, buffer, count, 10);
}

void afe4404_Delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

void afe4404_send_results(uint8_t num, uint16_t hr, uint32_t led1, uint32_t led2, uint32_t led3)
{
	char send_buf[128];
	sprintf(send_buf, "%d, %ld, %ld, %ld, %ld\r\n", num, hr, led1, led2, led3);
	CDC_Transmit_FS((unsigned char*) send_buf, strlen(send_buf));
}

void afe4404_send_noresult()
{
	char send_buf[128];
	sprintf(send_buf, "No result\n");
	CDC_Transmit_FS((unsigned char*) send_buf, strlen(send_buf));
}
