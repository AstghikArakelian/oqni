/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : f103.c
  * @brief          : Functions for communicating with ob1203
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
unsigned char* send_buf;
/* Private user code ---------------------------------------------------------*/
void ob1203_RstSet(void)
{
	HAL_GPIO_WritePin(ob1203_RST_GPIO_PORT, ob1203_RST_PIN, SET);
}

void ob1203_RstReset(void)
{
	HAL_GPIO_WritePin(ob1203_RST_GPIO_PORT, ob1203_RST_PIN, RESET);
}

HAL_StatusTypeDef ob1203_I2C_Write(uint8_t address, uint8_t * data, uint8_t count)
{
	return HAL_I2C_Master_Transmit(&cur_i2c, address << 1, data, count, 10);
}

HAL_StatusTypeDef ob1203_I2C_Read(uint8_t address, uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count)
{
	return HAL_I2C_Mem_Read(&cur_i2c, address << 1, *reg, cmd_size, buffer, count, 10);
}

void tca9544a_I2C_SetX(uint8_t slave)
{
	slave = slave | 4;
	HAL_I2C_Master_Transmit(&cur_i2c, tca9544a_address << 1, &slave, 1, 10);
}

void tca9544a_I2C_ReadX(uint8_t* data)
{
	HAL_I2C_Master_Receive(&cur_i2c, tca9544a_address << 1 | 1, data, 1, 10);
}

void ob1203_Delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

void ob1203_send_results(uint32_t ppg, unsigned char channel_num)
{
	send_buf = &ppg;
	ob1203_Delay_ms(2);
	ob1203_send_preambula();
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) send_buf, 4);
}

void BMX_send_result(int16_t* acc, int16_t* giro, int16_t* mag)
{
	ob1203_send_preambula();
	ob1203_Delay_ms(2);
	uint8_t channel_num = 4;
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) acc, 6);
	ob1203_Delay_ms(2);
	channel_num = 5;
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) giro, 6);
	ob1203_Delay_ms(2);
	channel_num = 6;
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) mag, 6);

}

void ob1203_send_preambula()
{
	char preambula[4];
	preambula[0] = 0xAA;
	preambula[1] = 0x55;
	preambula[2] = 0xAA;
	preambula[3] = 0x55;
	CDC_Transmit_FS((unsigned char*) &preambula, 4);
}

void ob1203_send_info(uint8_t rate)
{
	uint8_t size = 4;
	uint8_t channel_num = 1;
	ob1203_send_preambula();
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &rate, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &size, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
}

void bmx055_send_info(uint8_t rate)
{
	uint8_t size = 6;
	uint8_t channel_num = 1;
	ob1203_send_preambula();
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &rate, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &size, 1);
	ob1203_Delay_ms(2);
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);
}
