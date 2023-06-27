/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "command_decoder.h"
#include "gps.h"

#define UART_DEVICE_NODE DT_NODELABEL(uart1)

#define MSG_SIZE 128

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
static int old_fix_state = 0;

struct Position {
   char  latitude[10];
   char  lati_indicator;
   char  longitude[11];
   char  long_indicator;
};
struct DateTime {
   char  date[7];
   char  time[11];
};

static struct Position Curr_Pos;
static struct DateTime Curr_DateTime;

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c = 0;

	if (!uart_irq_update(uart_dev)) 
	{
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) 
	{

		uart_fifo_read(uart_dev, &c, 1);
		/*GMM-U2P has	<CR> <LF> line termination*/
		if ((c == '\n') && rx_buf_pos > 0) 
		{
			//add termination symbol to string
			rx_buf[rx_buf_pos] = '\0';
			char msg[MSG_SIZE];
			strcpy(msg, rx_buf);
			decode_message(msg); //decode after getting a full line
			rx_buf_pos = 0;
		} 
		else if (rx_buf_pos < (sizeof(rx_buf) - 1)) 
		{
			rx_buf[rx_buf_pos++] = c;
		}

	}
}

void decode_message(const char *message)
{
	char field[16];
    if (strncmp (message,"$GPRMC",6) == 0)
	{ //decode date and time
		extract_field(message, 1, field); //UTC time hhmmss.sss
		strcpy(Curr_DateTime.time, field);
		extract_field(message, 9, field); //Date ddmmyy
		strcpy(Curr_DateTime.date, field);
    }
	/*$GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65*/
	if (strncmp (message,"$GPGGA",6) == 0)
	{ //decode position, field 6 tells if there is a (0) no fix (1)fix (2) differential fix
		extract_field(message, 6, field);
		int fix;
		if (strcmp(field, "1") == 0)
		{
			fix = 1;
			encode_gps_info(0, "Fix");
		}
		else if (strcmp(field, "2") == 0)
		{
			fix = 2;
			encode_gps_info(0, "DiFix");
		}
		else
		{
			fix = 0;
			encode_gps_info(0, "No fix");
		}
		if (fix > 0)
		{
			if (fix != old_fix_state)
			{
				printk("Got GPS fix\n");
				old_fix_state = fix;
			}
			extract_field(message, 2, field); //Latitude ddmm.mmmm
			strcpy(Curr_Pos.latitude, field);
			extract_field(message, 3, field); //N/S
			Curr_Pos.lati_indicator = field[0];
			extract_field(message, 4, field); //Longitude dddmm.mmmm
			strcpy(Curr_Pos.longitude, field);
			extract_field(message, 5, field); //E/W
			Curr_Pos.long_indicator = field[0];
		}
		else
		{
			if (fix != old_fix_state)
			{
				printk("Lost GPS fix\n");
				old_fix_state = fix;
			}
		}
    }
}

void extract_field(const char *message, const uint8_t field_idx, char *field)
{
	//printk("Message %s\n", message);
	uint8_t curr_field_idx = 0;
	int idx = 0;
	int begin_idx = 0;
	int end_idx = 0;
	while (message[idx] != '\0') 
	{
		char curr_char = message[idx];
		if (curr_char == ',')
		{
			curr_field_idx++;
			if (curr_field_idx == field_idx)
			{
				begin_idx = idx + 1;
			}
			else if (curr_field_idx == (field_idx + 1))
			{
				end_idx = idx;
			}
		} 
		idx++;
	} 
	if (end_idx > begin_idx)
	{   //copy part of string between two commas
		strncpy(field, message + begin_idx, end_idx - begin_idx);
		/*char * strncpy( char * dest, const char * src, size_t count );
		strncpy adds \0 only if count is smaller than src, what is not true in our case*/
		field[end_idx - begin_idx] = '\0'; 
		//printk("Field id: %u Field: %s Strlen: %d\n", field_idx, field, strlen(field));
	}

}

void initialise_gps(void)
{

	if (!device_is_ready(uart_dev)) {
		printk("UART GPS device not found!\n");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);
	printk("UART GPS initialised\n");

}

void send_date_time(void)
{
   char  date[7];
   char  time[7];
   strcpy(date, Curr_DateTime.date);
   strncpy(time, Curr_DateTime.time, 6); //copy hhmmss without miliseconds
   time[6] = '\0';
   encode_gps_info(1, date);
   encode_gps_info(2, time);
}
void send_position(void)
{
	//2307.1256,N,12016.4438,E,
	char latit0[5];
	char latit1[6];
	char longit0[6];
	char longit1[6];
	strncpy(latit0, Curr_Pos.latitude, 4);
	latit0[4] = '\0';
	strncpy(latit1, Curr_Pos.latitude + 5 , 4);
	latit1[4] = Curr_Pos.lati_indicator;
	latit1[5] = '\0';
	encode_gps_info(3, latit0);
    encode_gps_info(4, latit1);

	strncpy(longit0, Curr_Pos.longitude, 5);
	longit0[5] = '\0';
	strncpy(longit1, Curr_Pos.longitude + 6 , 4);
	longit1[4] = Curr_Pos.long_indicator;
	longit1[5] = '\0';
	encode_gps_info(5, longit0);
    encode_gps_info(6, longit1);
}
