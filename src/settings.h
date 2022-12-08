/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef SETTINGS_H
#define SETTINGS_H

void initialise_settings(void);
void read_drive_settings(uint8_t drive_id, bool copy_to_drv);
void write_drive_settings(uint8_t drive_id);

#endif