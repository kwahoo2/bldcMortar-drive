/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

int initialise_gpio(void);
int initialise_i2c(void);
int set_motor_speed_dir(const uint16_t speed, const uint8_t dir, const uint8_t id);
int write_reg(const uint8_t addr, const uint8_t val, const uint8_t id);
int read_reg(const uint8_t reg_addr, const uint8_t id, uint8_t *reg_value_ptr);
int conf_reg_write_unlock(const uint8_t id);
void check_status_reg(const uint8_t drive_id);
void check_fault_reg(const uint8_t drive_id);
void read_motor_speed(const uint8_t drive_id, uint8_t *speed1_val_ptr, uint8_t *speed2_val_ptr);
void set_uart_driver_verbosity(const int level);
void read_and_send_all_regs(const uint8_t drive_id);
void probe_motor_lock(const uint8_t drive_id);
void reset_motor_lock_flag_and_info(const uint8_t drive_id);

#endif