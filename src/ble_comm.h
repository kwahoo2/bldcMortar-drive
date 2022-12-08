/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <zephyr/types.h>

void initialise_ble(void);
void send_enc0_ble_data(void);
void set_enc0_val(int64_t *val);
void send_enc1_ble_data(void);
void set_enc1_val(int64_t *val);
int send_command_ble_data(void);
void set_command_val(uint64_t *val);

#endif