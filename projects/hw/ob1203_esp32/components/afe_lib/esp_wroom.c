
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : f103.c
  * @brief          : Functions for communicating with ob1203
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <esp_wroom.h>



/* Private variables ---------------------------------------------------------*/
unsigned char* send_buf;
/* Private user code ---------------------------------------------------------*/

esp_err_t ob1203_I2C_Write(uint8_t address, uint8_t * data, uint8_t count)
{
	return i2c_master_write_to_device(0, address, data, count, 10);
}


esp_err_t ob1203_I2C_Read(uint8_t address, uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count)
{
	return i2c_master_write_read_device(0, address, reg, cmd_size, buffer, count, 10);
}


void tca9544a_I2C_SetX(uint8_t slave)
{
	slave = slave | 4;
	i2c_master_write_to_device(0, tca9544a_address, &slave, 1, 10);
}

void tca9544a_I2C_ReadX(uint8_t* data)
{
	i2c_master_read_from_device(0, tca9544a_address, data, 1, 10);

}
void ob1203_Delay_ms(uint32_t ms)
{
	vTaskDelay(ms/portTICK_PERIOD_MS);
}


void ob1203_send_results(uint32_t ppg, unsigned char channel_num)
{
	send_buf = &ppg;
	ob1203_send_preambula();
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
	uart_write_bytes(0, (unsigned char*) send_buf, 4);
}

void BMX_send_result(int16_t* acc, int16_t* giro, int16_t* mag)
{
	ob1203_send_preambula();
	uint8_t channel_num = 4;
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
	uart_write_bytes(0, (unsigned char*) acc, 6);
	channel_num = 5;
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
	uart_write_bytes(0, (unsigned char*) giro, 6);
	channel_num = 6;
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
	uart_write_bytes(0, (unsigned char*) mag, 6);

}


void ob1203_send_preambula()
{
	char preambula[4];
	preambula[0] = 0xAA;
	preambula[1] = 0x55;
	preambula[2] = 0xAA;
	preambula[3] = 0x55;
	uart_write_bytes(0, (unsigned char*) &preambula, 4);
}

void ob1203_send_info(uint16_t rate)
{
	uint8_t size = 4;
	uint8_t channel_num = 1;
	ob1203_send_preambula();
	uart_write_bytes(0, (unsigned char*) &rate, 2);
	uart_write_bytes(0, (unsigned char*) &size, 1);
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
}


void bmx055_send_info(uint8_t rate)
{
	uint8_t size = 2;
	uint8_t channel_num = 3;
	ob1203_send_preambula();
	uart_write_bytes(0, (unsigned char*) &rate, 1);
	uart_write_bytes(0, (unsigned char*) &size, 1);
	uart_write_bytes(0, (unsigned char*) &channel_num, 1);
}



