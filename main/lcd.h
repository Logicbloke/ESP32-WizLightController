#include <stdio.h>
#include "string.h"
#include "driver/i2c.h"


// LCD1602
#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16

// Define I2C pins for ESP32-S3
#define I2C_MASTER_SCL_IO 17        // SCL on GPIO 17
#define I2C_MASTER_SDA_IO 18        // SDA on GPIO 18
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27


void i2c_init() {
    i2c_config_t bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &bus_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, bus_config.mode,
     I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
    0));
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_t, 4, 1000));
}

void lcd_send_cmd (char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	ESP_ERROR_CHECK(i2c_master_write_to_device (I2C_MASTER_NUM, LCD_ADDR, data_t, 4, 1000));
}

void lcd_init (void)
{
	// 4 bit initialisation
	vTaskDelay(pdMS_TO_TICKS(100));  
	lcd_send_cmd (0x30);
	vTaskDelay(pdMS_TO_TICKS(100));  
	lcd_send_cmd (0x30);
	vTaskDelay(pdMS_TO_TICKS(100)); 
	lcd_send_cmd (0x30);
	vTaskDelay(pdMS_TO_TICKS(100));
	lcd_send_cmd (0x20);  // 4bit mode
	vTaskDelay(pdMS_TO_TICKS(100));

    // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	vTaskDelay(pdMS_TO_TICKS(100));
	lcd_send_cmd (0x01);  // clear display
	vTaskDelay(pdMS_TO_TICKS(300));
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	vTaskDelay(pdMS_TO_TICKS(100));
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	vTaskDelay(pdMS_TO_TICKS(100));
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}

void lcd_send_string (char *str)
{
	while(*str) lcd_send_data(*str++);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	vTaskDelay(pdMS_TO_TICKS(1000));
}