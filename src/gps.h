/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef GPS_H
#define GPS_H

#include <zephyr/types.h>

void initialise_gps(void);
void decode_message(const char *message);
void extract_field(const char *message, const uint8_t field_idx, char *field);
void send_date_time(void);
void send_position(void);

#endif