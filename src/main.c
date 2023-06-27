/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include "ble_comm.h"
#include "encoder_reader.h"
#include "bldc_driver.h"
#include "usb_comm.h"
#include "command_decoder.h"
#include "settings.h"
#include "gps.h"

#define MAIN_SLEEP_TIME_MS  100
#define MSG_SLEEP_TIME_MS  20
#define LOOP_DIVIDER 100

void main(void)
{
	initialise_usb_cdc();
	initialise_ble();
	initialise_encoder();
	initialise_gpio();
	initialise_i2c();
	initialise_settings();
	initialise_gps();

	int64_t old_enc0_count = 0;
	int64_t enc0_count = 0;
	int64_t old_enc1_count = 0;
	int64_t enc1_count = 0;
	int64_t old_fg0_count = 0;
	int64_t fg0_count = 0;
	int64_t old_fg1_count = 0;
	int64_t fg1_count = 0;

	extern int driver_verbosity;

	int cnt = 0;

	/*load regs from flash*/
	for (int id = 0; id < 2; ++id)
	{
		read_drive_settings(id, 1);
	}
	while (1) 
	{
		k_msleep(MAIN_SLEEP_TIME_MS);
		int msg_count;
		msg_count = send_command_ble_data();
		while (msg_count > 0)
		{
			k_msleep(MSG_SLEEP_TIME_MS);
			msg_count = send_command_ble_data(); //send command waiting in queue
		}
		get_enc0_val(&enc0_count);
		get_enc1_val(&enc1_count);
		get_fg0_val(&fg0_count);
		get_fg1_val(&fg1_count);
		for (int id = 0; id < 2; ++id)
		{
			probe_motor_lock(id);
		}

		if (enc0_count != old_enc0_count)
		{
			if (driver_verbosity > 2) {printk("Encoder 0 steps count  %" PRId64 "\n", enc0_count);}
			set_enc0_val(&enc0_count); //read encoder value and send to ble
			send_enc0_ble_data(); //send notification
		}
		old_enc0_count = enc0_count;

		if (enc1_count != old_enc1_count)
		{
			if (driver_verbosity > 2) {printk("Encoder 1 steps count  %" PRId64 "\n", enc1_count);}
			set_enc1_val(&enc1_count);
			send_enc1_ble_data(); //send notification
		}
		old_enc1_count = enc1_count;

		if (fg0_count != old_fg0_count)
		{
			if (driver_verbosity > 2) {printk("FG0 count  %" PRId64 "\n", fg0_count);}
			encode_fg_value(fg0_count, 0);
		}
		old_fg0_count = fg0_count;

		if (fg1_count != old_fg1_count)
		{
			if (driver_verbosity > 2) {printk("FG1 count  %" PRId64 "\n", fg1_count);}
			encode_fg_value(fg1_count, 1);
		}
		old_fg1_count = fg1_count;

		cnt++;
		if (cnt > LOOP_DIVIDER)
		{
			send_date_time();
			send_position();
			for (int id = 0; id < 2; ++id)
			{
				check_status_reg(id);
				uint8_t speed1 = 0;
				uint8_t speed2 = 0;
				read_motor_speed(id, &speed1, &speed2);
				encode_motor_speed(speed1, speed2, id);

			}
			cnt = 0;
		}
	}	
}