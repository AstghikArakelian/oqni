#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/i2c.h"
#include "esp_wroom.h"
#include "heartrate_3.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"


#define ST_IDLE              (uint8_t)0x00
#define ST_START             (uint8_t)0x01
#define ST_STOP		         (uint8_t)0x02

#define BUF_SIZE (1024)
#define toggle_pin GPIO_NUM_2

//#define EX_UART_NUM UART_NUM_0
//#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define RD_BUF_SIZE (BUF_SIZE)
#define EX_UART_NUM UART_NUM_0
#define UART_TX 1ULL<<1
#define UART_RX 1ULL<<3

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

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

typedef struct {
    uint64_t event_count;
} example_queue_element_t;


BUF_HandleTypeDef FIFO_buf = 0;
uint8_t adc1_rdy = 0;
uint8_t adc2_rdy = 0;
i2c_port_t cur_i2c;
uint8_t command = -1;
uint8_t SM_Case = ST_IDLE;


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* buff = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
    	vTaskDelay(2);
	//Waiting for UART event.
		if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
			bzero(buff, RD_BUF_SIZE);
			if(event.type == UART_DATA){
				uart_read_bytes(EX_UART_NUM, buff, event.size, portMAX_DELAY);
				for (int i = 0; i < event.size; ++i)
				  {

					  Buffer_Write(FIFO_buf, buff[i]);
				  }
			}
		}
    }
    free(buff);
    buff = NULL;
    vTaskDelete(NULL);
}



int Command_Check()
{
	int command_count = COMMAND_FIRST_BYTE;
	int command_status = COMMAND_NOTRECEIVED;
	while (! Buffer_IsEmpty(FIFO_buf) && command_status == COMMAND_NOTRECEIVED)
		{
		uint8_t data = Buffer_Read(FIFO_buf);
		switch (command_count)
		{
		case COMMAND_FIRST_BYTE:
			if(data == 0){
				command_count = 1;

			}
			else if(data == 255){
				command_count = 2;
			}
			else{
				command_count = 0;
			}
			break;
		case COMMAND_SECOND_BYTE:
			if(data == 7){
				command_status = COMMAND_RECEIVED;
				command = COMMAND_START;
			}
			else{
				command_status =  COMMAND_NOTRECEIVED;
			}
			break;
		case COMMAND_SWITCH:
			if(data == 0){
				command_status = COMMAND_RECEIVED;
				command = COMMAND_STOP;
			}
			else{
				 command_status = COMMAND_NOTRECEIVED;
			}
			break;
		}
	}
	return command_status;
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
	    io_conf.pin_bit_mask = 1ULL<<afe4404_GP2INT_PIN;
	    io_conf.mode = GPIO_MODE_INPUT;
//	    io_conf.pull_up_en = 1;
	    io_conf.pull_down_en = 1;
	    gpio_config(&io_conf);

	    io_conf.intr_type = GPIO_INTR_POSEDGE;
	    io_conf.pin_bit_mask = 1ULL<<afe4404_GP1INT_PIN;
	    io_conf.mode = GPIO_MODE_INPUT;
//	    io_conf.pull_up_en = 1;
	    io_conf.pull_down_en = 1;
	    gpio_config(&io_conf);

	    gpio_evt_queue = xQueueCreate( 10, sizeof( unsigned long ) );
		gpio_install_isr_service(0);
	    gpio_isr_handler_add(afe4404_GP1INT_PIN, gpio1_isr_handler, (void*) afe4404_GP1INT_PIN);
	    gpio_isr_handler_add(afe4404_GP2INT_PIN, gpio2_isr_handler, (void*) afe4404_GP2INT_PIN);

}

static void configure_led(){
    gpio_reset_pin(toggle_pin);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(toggle_pin, GPIO_MODE_OUTPUT);
}

static esp_err_t i2c1_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
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
    int i2c_master_port = I2C_NUM_1;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_19,
        .scl_io_num = GPIO_NUM_18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


static void uart_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_pattern_queue_reset(EX_UART_NUM, 20);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 15, NULL);
}


static void AFE12_RST(void)
{
	afe4404_RstReset();
	afe4404_Delay_ms(0.05);
	afe4404_RstSet();
}

static void AFE_Setup(void)
{
	dynamic_modes_t dynamic_modes;

	dynamic_modes.transmit = trans_en;
	dynamic_modes.curr_range = led_norm;
	dynamic_modes.adc_power = adc_on;
	dynamic_modes.clk_mode = osc_mode;
	dynamic_modes.tia_power = tia_off;
	dynamic_modes.rest_of_adc = rest_of_adc_off;
	dynamic_modes.afe_rx_mode = afe_rx_normal;
	dynamic_modes.afe_mode = afe_normal;
	hr3_init(afe4404_address, &dynamic_modes);
}


void app_main(void)
{
	FIFO_buf = Buffer_Init(128);
	gpio_init();
	i2c1_master_init();
	configure_led();
	uart_init();
	i2c2_master_init();
	int setup = SETUP_NOTDONE;
	command = COMMAND_NOTR;

	uint32_t led1 = 0;
	uint32_t led2 = 0;
	uint32_t led3 = 0;

	adc1_rdy = 0;
	adc2_rdy = 0;
	afe4404_Delay_ms(200);
	initStatHRM();
	while (true) {
		vTaskDelay(2);
		if(Command_Check() == COMMAND_RECEIVED)
		{
			SM_Case = ST_IDLE;
		}
		switch(SM_Case)
		{
			case ST_IDLE:
				if (command == COMMAND_START)
				{
					SM_Case = ST_START;
				}
				if (command == COMMAND_STOP)
				{
					SM_Case = ST_STOP;
				}
				break;
			case ST_START:
				if (setup == SETUP_NOTDONE)
				{
					AFE12_RST();
					cur_i2c = I2C_NUM_0;
					AFE_Setup();
					cur_i2c = I2C_NUM_1;
					AFE_Setup();
					setup = SETUP_DONE;
					gpio_set_level(toggle_pin, 1);

				}
				if(adc1_rdy)
				{
					 cur_i2c = I2C_NUM_0;
					 led2 = hr3_get_led2_val();
					 led3 = hr3_get_led3_val();
					 led1 = hr3_get_led1_val();
					 afe4404_send_results(1, led1, led2 , led3);
					 adc1_rdy = 0;
				}
				if(adc2_rdy)
				{
					 cur_i2c = I2C_NUM_1;
					 led2 = hr3_get_led2_val();
					 led3 = hr3_get_led3_val();
					 led1 = hr3_get_led1_val();
					 afe4404_send_results(2, led1, led2 , led3);
					 adc2_rdy = 0;
				}
				break;
			case ST_STOP:
				AFE12_RST();
				setup = SETUP_NOTDONE;
				break;

		}
    }
}
