
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : f103.c
  * @brief          : Functions for communicating with afe4404
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <esp_wroom.h>



/* Private variables ---------------------------------------------------------*/
extern i2c_port_t cur_i2c;

/* Private user code ---------------------------------------------------------*/
void afe4404_RstSet(void)
{
	gpio_set_level(afe4404_RST_PIN, 1);
}

void afe4404_RstReset(void)
{
	gpio_set_level(afe4404_RST_PIN, 0);
}

void afe4404_I2C_Write(uint8_t * data, uint8_t count)
{
	i2c_master_write_to_device(cur_i2c, afe4404_address, data, count, 10);
}


void afe4404_I2C_Read(uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count)
{
	i2c_master_write_read_device(cur_i2c, afe4404_address, reg, cmd_size, buffer, count, 10);
}

void afe4404_Delay_ms(uint32_t ms)
{
	vTaskDelay(ms);
}

void afe4404_send_noresult()
{
	printf("No result\n");
}


void afe4404_send_results(uint8_t num,uint32_t led1, uint32_t led2, uint32_t led3)
{
	unsigned char* send_buf;
	send_buf = (unsigned char*)&led1;
	afe4404_send_preambula();
	uart_write_bytes(0, (unsigned char*) &num, 1);

	uart_write_bytes(0, (unsigned char*) send_buf, 4);
//	vTaskDelay(0.1);
	send_buf = (unsigned char*)&led2;
	uart_write_bytes(0, (unsigned char*) send_buf, 4);
//	vTaskDelay(0.1);
	send_buf = (unsigned char*)&led3;
	uart_write_bytes(0, (unsigned char*) send_buf, 4);
//	vTaskDelay(0.1);
}

void afe4404_send_preambula()
{
	char preambula[4];
	preambula[0] = 0xAA;
	preambula[1] = 0x55;
	preambula[2] = 0xAA;
	preambula[3] = 0x55;
	uart_write_bytes(0, (unsigned char*) &preambula, 4);
//	vTaskDelay(0.1);
}
