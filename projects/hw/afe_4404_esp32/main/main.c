#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/i2c.h"
#include "esp_wroom.h"
#include "heartrate_3.h"
#include "driver/gpio.h"

typedef enum
{
  PREAMBULA_RECEIVED = 0U,
  PREAMBULA_NOTRECEIVED
} Preambula_StatusTypeDef;

typedef enum
{
  PREAMBULA_FIRST_BYTE = 0U,
  PREAMBULA_SECOND_BYTE,
  PREAMBULA_THIRD_BYTE,
  PREAMBULA_FORTH_BYTE,
  PREAMBULA_FIFTH_DUMMY_BYTE,
  PREAMBULA_SIXTH_DUMMY_BYTE
} Preambula_ByteCheckTypeDef;



BUF_HandleTypeDef FIFO_buf = 0;
uint8_t adc1_rdy = 0;
uint8_t adc2_rdy = 0;
i2c_port_t cur_i2c;

int Preambula_Check()
{
	int preambula_count = PREAMBULA_FIRST_BYTE;
	int preambula_status = PREAMBULA_NOTRECEIVED;
	while (! Buffer_IsEmpty(FIFO_buf) && preambula_status == PREAMBULA_NOTRECEIVED)
	{
		uint8_t data = Buffer_Read(FIFO_buf);
		switch (preambula_count)
		{
		case PREAMBULA_FIRST_BYTE:
			preambula_count = (data == 0xaa)? 1: 0;
			break;
		case PREAMBULA_SECOND_BYTE:
			preambula_count = (data == 0x55)? 2: 0;
			break;
		case PREAMBULA_THIRD_BYTE:
			preambula_count = (data == 0xaa)? 3: 0;
			break;
		case PREAMBULA_FORTH_BYTE:
			preambula_count = (data == 0x55)? 4: 0;
			break;
		case PREAMBULA_FIFTH_DUMMY_BYTE:
			preambula_count = 5;
			break;
		case PREAMBULA_SIXTH_DUMMY_BYTE:
			preambula_status = PREAMBULA_RECEIVED;
			break;
		}
	}
	return preambula_status;
}

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio1_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    adc1_rdy = 1;
}

static void IRAM_ATTR gpio2_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    adc2_rdy = 1;
}


static void gpio_init(){
	   //zero-initialize the config structure.
	    gpio_config_t io_conf = {};
	    io_conf.intr_type = GPIO_INTR_DISABLE;
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    io_conf.pin_bit_mask = 1ULL<<afe4404_RST_PIN;
	    io_conf.pull_down_en = 0;
	    io_conf.pull_up_en = 0;
	    gpio_config(&io_conf);

	    io_conf.intr_type = GPIO_INTR_POSEDGE;
	    io_conf.pin_bit_mask = 1ULL<<afe4404_GP1INT_PIN;
	    io_conf.mode = GPIO_MODE_INPUT;
	    io_conf.pull_up_en = 0;
	    gpio_config(&io_conf);


	    io_conf.intr_type = GPIO_INTR_POSEDGE;
	    io_conf.pin_bit_mask = 1ULL<<afe4404_GP2INT_PIN;
	    io_conf.mode = GPIO_MODE_INPUT;
	    io_conf.pull_up_en = 0;
	    gpio_config(&io_conf);
	    gpio_evt_queue = xQueueCreate( 10, sizeof( unsigned long ) );
		gpio_install_isr_service(0);
	    gpio_isr_handler_add(afe4404_GP1INT_PIN, gpio1_isr_handler, (void*) afe4404_GP1INT_PIN);
	    gpio_isr_handler_add(afe4404_GP2INT_PIN, gpio2_isr_handler, (void*) afe4404_GP2INT_PIN);

}



static esp_err_t i2c1_master_init(void)
{
    int i2c_master_port = 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


static esp_err_t i2c2_master_init(void)
{
    int i2c_master_port = 1;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 1ULL<<GPIO_NUM_18,
        .scl_io_num = 1ULL<<GPIO_NUM_19,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static void AFE12_RST(void)
{
	afe4404_RstReset();
	afe4404_Delay_ms(0.05);
	afe4404_RstSet();
}

static void AFE1_Setup(void)
{
	dynamic_modes_t dynamic_modes;

	dynamic_modes.transmit = trans_dis;
	dynamic_modes.curr_range = led_double;
	dynamic_modes.adc_power = adc_on;
	dynamic_modes.clk_mode = osc_mode;
	dynamic_modes.tia_power = tia_off;
	dynamic_modes.rest_of_adc = rest_of_adc_off;
	dynamic_modes.afe_rx_mode = afe_rx_normal;
	dynamic_modes.afe_mode = afe_normal;

	cur_i2c = I2C_NUM_0;
	hr3_init(afe4404_address, &dynamic_modes);
}

static void AFE2_Setup(void)
{
	dynamic_modes_t dynamic_modes;

	dynamic_modes.transmit = trans_dis;
	dynamic_modes.curr_range = led_double;
	dynamic_modes.adc_power = adc_on;
	dynamic_modes.clk_mode = osc_mode;
	dynamic_modes.tia_power = tia_off;
	dynamic_modes.rest_of_adc = rest_of_adc_off;
	dynamic_modes.afe_rx_mode = afe_rx_normal;
	dynamic_modes.afe_mode = afe_normal;

	cur_i2c = I2C_NUM_1;
	hr3_init(afe4404_address, &dynamic_modes);
}


void app_main(void)
{
	FIFO_buf = Buffer_Init(128);
	printf("1\n");
	gpio_init();
	printf("2\n");
	//ESP_LOGI("gpio init done");
	i2c1_master_init();
	printf("1\n");
	//i2c2_master_init();
	printf("3\n");
	AFE12_RST();
	AFE1_Setup();
	AFE2_Setup();
	adc1_rdy = 0;
	adc2_rdy = 0;
	printf("4\n");
	afe4404_Delay_ms(200);
	printf("5\n");
	initStatHRM();
	printf("6\n");
	while (true) {
		 if(adc1_rdy)
		 {
			 cur_i2c = I2C_NUM_0;
			 statHRMAlgo(hr3_get_led1_amb1_val());
			 afe4404_send_results(1, hr3_get_heartrate(), hr3_get_led1_val(), hr3_get_led2_val(), hr3_get_led3_val());
			 adc1_rdy = 0;
		 }
		 if(adc2_rdy)
		 {
			 cur_i2c = I2C_NUM_1;
			 statHRMAlgo(hr3_get_led1_amb1_val());
			 afe4404_send_results(2, hr3_get_heartrate(), hr3_get_led1_val(), hr3_get_led2_val(), hr3_get_led3_val());
			 adc2_rdy = 0;
		 }
    }
}
