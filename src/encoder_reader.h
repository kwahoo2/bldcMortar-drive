/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <zephyr/types.h>

void initialise_encoder(void);
void get_enc0_val(int64_t *steps);
void get_enc1_val(int64_t *steps);
void get_fg0_val(int64_t *steps);
void get_fg1_val(int64_t *steps);
void set_fg_dir(uint8_t fg_id, uint8_t dir);

#endif