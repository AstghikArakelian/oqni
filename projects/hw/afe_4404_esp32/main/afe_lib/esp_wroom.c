
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
	i2c_master_write_to_device(cur_i2c, afe4404_address<< 1, data, count, 10);
}

void afe4404_I2C_Read(uint8_t * reg, uint8_t * buffer, uint8_t cmd_size, uint8_t count)
{
	i2c_master_write_read_device(cur_i2c, afe4404_address<< 1, *reg, cmd_size, buffer, count, 10);
}

void afe4404_Delay_ms(uint32_t ms)
{
	//HAL_Delay(ms);
	ets_delay_us(100*ms);
}

void afe4404_send_results(uint8_t num, uint16_t hr, uint32_t led1, uint32_t led2, uint32_t led3)
{
	char send_buf[128];
	printf(send_buf, "%d, %ld, %ld, %ld, %ld\r\n", num, hr, led1, led2, led3);
}

void afe4404_send_noresult()
{
	char send_buf[128];
	printf(send_buf, "No result\n");
}
