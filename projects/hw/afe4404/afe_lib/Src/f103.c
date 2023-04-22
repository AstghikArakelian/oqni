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
	unsigned char channel_num = 3;
	afe4404_send_preambula();
	afe4404_Delay_ms(5);

	CDC_Transmit_FS((unsigned char*) &num, 1);
	afe4404_Delay_ms(5);
	CDC_Transmit_FS((unsigned char*) &channel_num, 1);

	afe4404_Delay_ms(5);
	CDC_Transmit_FS((unsigned char*) &led1, 4);
	afe4404_Delay_ms(5);

	CDC_Transmit_FS((unsigned char*) &led2, 4);
	afe4404_Delay_ms(5);

	CDC_Transmit_FS((unsigned char*) &led3, 4);

	//	char send[128];
	//	sprintf(send, "%d, %ld\r\n", num, led1);
	//	CDC_Transmit_FS((unsigned char*) send, strlen(send));
	//
}

void afe4404_send_preambula()
{
	char preambula[4];
	preambula[0] = 0xAA;
	preambula[1] = 0x55;
	preambula[2] = 0xAA;
	preambula[3] = 0x55;
	CDC_Transmit_FS((unsigned char*) &preambula, 4);
}
