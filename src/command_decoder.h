/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef COMMAND_DECODER_H
#define COMMAND_DECODER_H

#include <zephyr/types.h>

void decode_and_run (uint64_t *command);
void encode_send_reg(uint8_t reg_addr, uint8_t reg_value, uint8_t drive_id);
void encode_send_flash_reg(uint8_t reg_addr, uint8_t reg_value, uint8_t drive_id);
void encode_motor_speed(uint8_t speed1, uint8_t speed2, uint8_t drive_id);
void encode_fg_value(int64_t fg_count, uint8_t drive_id);
void encode_gps_info(uint8_t info_type, char *info);

#endif