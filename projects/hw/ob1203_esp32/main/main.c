#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/i2c.h"
#include "esp_wroom.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heartrate11.h"
#include "bmx055.h"
#include "driver/gptimer.h"


#define RD_BUF_SIZE (BUF_SIZE)
#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define toggle_pin GPIO_NUM_2
static QueueHandle_t uart0_queue;


#define ST_IDLE              (uint8_t)0x00
#define ST_START             (uint8_t)0x01
#define ST_STOP		         (uint8_t)0x02



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

typedef enum
{
  BMX055_OK = 0U,
  BMX055_ERROR
} BMX_StatusTypeDef;

typedef enum
{
  I2C_1 = 0U,
  I2C_2,
  I2C_3
} OB1203X_CheckTypeDef;

typedef enum
{
  I2C1_INT = 1U,
  I2C2_INT,
  I2C3_INT
} OB1203XINT_CheckTypeDef;

typedef struct {
    uint64_t event_count;
} example_queue_element_t;

uint8_t SM_Case = ST_IDLE;
BUF_HandleTypeDef FIFO_buf = 0;

uint8_t command = 2;
uint8_t rate;
uint8_t adc_rdy = 0;

uint8_t BMX_rdy = 0;



static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    //size_t buffered_size;
    uint8_t* buff = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
    	vTaskDelay(2);
	//Waiting for UART event.
		if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
			bzero(buff, RD_BUF_SIZE);
			if(event.type == UART_DATA){
				//ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
				uart_read_bytes(EX_UART_NUM, buff, event.size, portMAX_DELAY);
				//uart_write_bytes(EX_UART_NUM, (const char*) buff, event.size);
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
    adc_rdy = 1;
}


static void gpio_init(){
    gpio_config_t io_conf = {};
	    io_conf.intr_type = GPIO_INTR_NEGEDGE;
	    io_conf.pin_bit_mask = ob1203_GP1INT_PIN;
	    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
//	    io_conf.pull_up_en = 1;
	    io_conf.pull_down_en = 1;
	    gpio_config(&io_conf);

	    gpio_evt_queue = xQueueCreate( 10, sizeof( unsigned long ) );
		gpio_install_isr_service(0);
	    gpio_isr_handler_add(ob1203_GP1INT_PIN, gpio1_isr_handler, (void*) ob1203_GP1INT_PIN);
}

static void configure_led(){
    gpio_reset_pin(2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
}

static esp_err_t i2c1_master_init(void)
{
    int i2c_master_port = 0;

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



static bool IRAM_ATTR timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(queue, &ele, &high_task_awoken);
    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 10000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static void timer_init()
{
    example_queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!queue) {
        return;
    }
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, queue);
    gptimer_enable(gptimer);
    gptimer_alarm_config_t alarm_config3 = {
        .alarm_count = 10000, // period = 1s
    };
    gptimer_set_alarm_action(gptimer, &alarm_config3);
    gptimer_start(gptimer);
    for (;;) {
        if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000))) {
        	BMX_rdy = 1;
        }
    }
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
    //Set UART log level
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_pattern_queue_reset(EX_UART_NUM, 20);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 15, NULL);
}

void OB1203_RST(void)
{
	heartrate11_t heartrate11;
	heartrate11_reset_device(&heartrate11);
}

void OB1203_Setup(void)
{
	heartrate11_t heartrate11;
	uint8_t data = 0;
	if(HEARTRATE11_OK == heartrate11_read_register(0, HEARTRATE11_REG_PART_ID, &data))
	{
		heartrate11_default_cfg(&heartrate11);
	}
}

uint16_t rate_case(uint8_t rate)
{
	uint16_t rate_hz = 0;
	switch(rate)
	{
	case HEARTRATE11_PPG_RATE_1MS:
		rate_hz = 1000;
		break;
	case HEARTRATE11_PPG_RATE_1p25MS:
		rate_hz = 800;
		break;
	case HEARTRATE11_PPG_RATE_2p5MS:
		rate_hz = 400;
		break;
	case HEARTRATE11_PPG_RATE_5MS:
		rate_hz = 200;
		break;
	case HEARTRATE11_PPG_RATE_10MS:
		rate_hz = 100;
		break;
	case HEARTRATE11_PPG_RATE_20MS:
		rate_hz = 50;
		break;
	}
	return rate_hz;
}


void app_main(void)
{
	FIFO_buf = Buffer_Init(128);
	gpio_init();
	i2c1_master_init();
	configure_led();
	uart_init();
    xTaskCreate(timer_init, "timer_event_task", 2048, NULL, 12, NULL);
	adc_rdy = 0;
	uint32_t ppg;
	uint8_t tca_data;
	int setup = SETUP_NOTDONE;
	command = COMMAND_NOTR;

	while (1)
	  {
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
					rate = HEARTRATE11_PPG_RATE_1MS;
					tca9544a_I2C_SetX(I2C_1);
					OB1203_Setup();

					tca9544a_I2C_SetX(I2C_2);
					OB1203_Setup();

					tca9544a_I2C_SetX(I2C_3);
					OB1203_Setup();

					tca9544a_I2C_ReadX(&tca_data);
					tca_data = tca_data >> 4;

					ob1203_send_info(rate_case(rate));

					BMX055_init_globals();
					BMX055_setup();
					bmx055_send_info(100);
					gpio_set_level(2, 1);
					setup = SETUP_DONE;

				}
				if(!gpio_get_level(ob1203_GP1INT_PIN))
				{
					gpio_set_level(2, 0);
					tca9544a_I2C_ReadX(&tca_data);
					tca_data = tca_data >> 4;
					if(tca_data && I2C1_INT)
					{
						tca9544a_I2C_SetX(I2C_1);
						memset(&ppg, 0, sizeof(ppg));
						heartrate11_read_fifo(0, &ppg);
						ob1203_send_results(ppg, I2C1_INT);
					}
					if(tca_data && I2C2_INT)
					{
						tca9544a_I2C_SetX(I2C_2);
						memset(&ppg, 0, sizeof(ppg));
						heartrate11_read_fifo(0, &ppg);
						ob1203_send_results(ppg, I2C2_INT);
					}
					if(tca_data && I2C3_INT)
					{
						tca9544a_I2C_SetX(I2C_3);
						memset(&ppg, 0, sizeof(ppg));
						heartrate11_read_fifo(0, &ppg);
						ob1203_send_results(ppg, I2C3_INT);
					}
					adc_rdy = 0;
				}
				if (BMX_rdy)
				{
					readAccelData(accelCount);
					readGyroData(gyroCount);
					readMagData(magCount);
					BMX_send_result(accelCount, gyroCount, magCount);
					BMX_rdy = 0;
				}
				break;
			case ST_STOP:
				OB1203_RST();
				setup = SETUP_NOTDONE;
				break;
		}
	  }
}
