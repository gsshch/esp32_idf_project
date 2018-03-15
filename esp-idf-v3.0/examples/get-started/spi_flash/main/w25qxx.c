/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   21

#define MY_LED 2
#define BUF_SIZE (1024)

/* Private typedef -----------------------------------------------------------*/
//#define SPI_FLASH_PageSize      4096
#define SPI_FLASH_PageSize      256
#define SPI_FLASH_PerWritePageSize      256

/* Private define ------------------------------------------------------------*/
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB 
#define W25X_ManufactDeviceID   	0x90 
#define W25X_JedecDeviceID		    0x9F 

#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */

#define Dummy_Byte                0xFF

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_CS_LOW(void){
	gpio_set_level(PIN_NUM_CS, 0);
}

void SPI_FLASH_CS_HIGH(void){
	gpio_set_level(PIN_NUM_CS, 1);
}

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void spi_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                     //Command is 8 bits
	t.tx_buffer = &cmd;               //The data is the cmd itself
	t.user = (void*) 0;                //D/C needs to be set to 0
	ret=spi_device_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void spi_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
	esp_err_t ret;
	spi_transaction_t t;
	if (len == 0)
	return;             //no need to send anything
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = len * 8;        //Len is in bytes, transaction length is in bits.
	t.tx_buffer = data;               //Data
	t.user = (void*) 1;                //D/C needs to be set to 1
	ret = spi_device_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);
}
//char recvbuf[128]="";




void Write_Max7219(spi_device_handle_t spi, uint8_t address, uint8_t dat) {
	SPI_FLASH_CS_LOW();
	spi_cmd(spi, address);           //写入地址，即数码管编号
	uint8_t temp_data = dat;
	spi_data(spi, &temp_data, 1);               //写入数据，即数码管显示数字
	SPI_FLASH_CS_HIGH();
}


void Init_MAX7219(spi_device_handle_t spi) {
	Write_Max7219(spi, 0x09, 0xff);       //译码方式：BCD码
	Write_Max7219(spi, 0x0a, 0x03);       //亮度
	Write_Max7219(spi, 0x0b, 0x07);       //扫描界限；4个数码管显示
	Write_Max7219(spi, 0x0c, 0x01);       //掉电模式：0，普通模式：1
	Write_Max7219(spi, 0x0f, 0x01);       //显示测试：1；测试结束，正常显示：0
}

//void spi_task(void)
void spi_task(void *pvParameter)
{
	gpio_set_direction(MY_LED, GPIO_MODE_OUTPUT);
// configure spi
//esp_err_t ret;
	spi_device_handle_t spi;
	spi_bus_config_t buscfg = {
			.miso_io_num = PIN_NUM_MISO,
			.mosi_io_num = PIN_NUM_MOSI,
			.sclk_io_num = PIN_NUM_CLK,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1
	};
	spi_device_interface_config_t devcfg = {
			.clock_speed_hz = 10000000, //Clock out at 10 MHz
			.mode = 0, //SPI mode 0
			.spics_io_num = PIN_NUM_CS, //CS pin
			.queue_size = 7, //We want to be able to queue
// 			.pre_cb=ili_spi_pre_transfer_callback, //Specify pre-transfer call
			};
//Initialize the SPI bus
	spi_bus_initialize(HSPI_HOST, &buscfg, 1);
//assert(ret==ESP_OK);
//Attach the SPI bus
	spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
//assert(ret==ESP_OK);
//-- configure uart
	int uart_num = UART_NUM_0;
	uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122,
	};
//Configure UART0 parameters
	uart_param_config(uart_num, &uart_config);
	uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
	char data[40];
	uint8_t num1 = 20, num2 = 30, num3 = 40, num4 = 50;
	while (1) {
// prepare to data to be sent
		spi_transaction_t tx_trans[2];
		// data for sending
		memset(&tx_trans[0], 0, sizeof(spi_transaction_t));
		tx_trans[0].length = 8 * 4;
		tx_trans[0].user = (void*) 0;
		tx_trans[0].tx_data[0] = num1;
		tx_trans[0].tx_data[1] = num2;
		tx_trans[0].tx_data[2] = num3;
		tx_trans[0].tx_data[3] = num4;
		tx_trans[0].flags = SPI_TRANS_USE_TXDATA;
		// data for receiving
		memset(&tx_trans[1], 0, sizeof(spi_transaction_t));
		tx_trans[1].length = 8 * 4;
		tx_trans[1].user = (void*) 0;
		tx_trans[1].rx_data[0] = 0;
		tx_trans[1].rx_data[1] = 0;
		tx_trans[1].rx_data[2] = 0;
		tx_trans[1].rx_data[3] = 0;
		tx_trans[1].flags = SPI_TRANS_USE_RXDATA;
		// send data
		spi_device_queue_trans(spi, &tx_trans[0], portMAX_DELAY);
		spi_device_queue_trans(spi, &tx_trans[1], portMAX_DELAY);
		//assert(ret==ESP_OK);
		// wait to receive data
		spi_transaction_t *rtrans;
//		spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
		spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
		//assert(ret==ESP_OK);
		//Write data back to UART
		gpio_set_level(MY_LED, 1);
		memset(&data, 0, sizeof(data));
		sprintf(data, "SENT: %04x %04x %04x %04x\r\n", num1, num2, num3, num4);
		uart_write_bytes(uart_num, (const char*) data, sizeof(data));
		memset(&data, 0, sizeof(data));
		sprintf(data, "RCV: %04x %04x %04x %04x\r\n", rtrans->rx_data[0],
				rtrans->rx_data[1], rtrans->rx_data[2], rtrans->rx_data[3]);
//		sprintf(data, "RCV: %04x %04x %04x %04x\r\n", tx_trans[1].rx_data[0], tx_trans[1].rx_data[1],
//				tx_trans[1].rx_data[2], tx_trans[1].rx_data[3]);
		uart_write_bytes(uart_num, (const char*) data, sizeof(data));
		gpio_set_level(MY_LED, 0);
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		num1++;
		num2++;
		num2++;
		num4++;
		if (num1 > 50)
			num1 = 20;
		if (num2 > 60)
			num2 = 30;
		if (num3 > 70)
			num1 = 40;
		if (num4 > 80)
			num1 = 50;
	}
}


void app_main()
{
	//spi_task();
	xTaskCreate(&spi_task, "spi_task", 1024 * 4, NULL, 10, NULL);
}
