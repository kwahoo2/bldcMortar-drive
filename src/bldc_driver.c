/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <drivers/i2c.h>

#include <sys/printk.h>

#include "bldc_driver.h"
#include "ble_comm.h"
#include "command_decoder.h"

#define I2C0_NODE DT_NODELABEL(i2c0)
#if DT_NODE_HAS_STATUS(I2C0_NODE, okay)
#define I2C0	DT_LABEL(I2C0_NODE)
#else
/* A build error here means your board does not have I2C enabled. */
#error "i2c0 devicetree node is disabled"
#define I2C0	""
#endif

#define I2C1_NODE DT_NODELABEL(i2c1)
#if DT_NODE_HAS_STATUS(I2C1_NODE, okay)
#define I2C1	DT_LABEL(I2C1_NODE)
#else
#error "i2c1 devicetree node is disabled"
#define I2C1	""
#endif

#define DIR0_NODE    DT_ALIAS(dir0)
#if DT_NODE_HAS_STATUS(DIR0_NODE, okay)
#define DIR0_GPIO_LABEL  DT_GPIO_LABEL(DIR0_NODE, gpios)
#define DIR0_GPIO_PIN    DT_GPIO_PIN(DIR0_NODE, gpios)
#define DIR0_GPIO_FLAGS  DT_GPIO_FLAGS(DIR0_NODE, gpios)
#else
#error "Unsupported board: dir0 devicetree alias is not defined"
#define DIR0_GPIO_LABEL  ""
#define DIR0_GPIO_PIN    0
#define DIR0_GPIO_FLAGS  0
#endif

#define DIR1_NODE    DT_ALIAS(dir1)
#if DT_NODE_HAS_STATUS(DIR1_NODE, okay)
#define DIR1_GPIO_LABEL  DT_GPIO_LABEL(DIR1_NODE, gpios)
#define DIR1_GPIO_PIN    DT_GPIO_PIN(DIR1_NODE, gpios)
#define DIR1_GPIO_FLAGS  DT_GPIO_FLAGS(DIR1_NODE, gpios)
#else
#error "Unsupported board: dir1 devicetree alias is not defined"
#define DIR1_GPIO_LABEL  ""
#define DIR1_GPIO_PIN    0
#define DIR1_GPIO_FLAGS  0
#endif

#define DRV10983_ADDRESS 0x52 //DRV 10983 adress
#define SPEEDCTRL1 0x00
#define SPEEDCTRL2 0x01
#define MOTORSPEED1 0x11
#define MOTORSPEED2 0x12
#define MOTORPARAM1 0x20
#define MOTORPARAM2 0x21
#define DEF_MPR 0x5D //motor phase resistance
#define DEF_BEMF 0x2E //BEMF constant

const struct device *dev_i2c0;
const struct device *dev_i2c1;

static const struct gpio_dt_spec dirpin0 = GPIO_DT_SPEC_GET(DIR0_NODE, gpios);
static const struct gpio_dt_spec dirpin1 = GPIO_DT_SPEC_GET(DIR1_NODE, gpios);

int driver_verbosity = 2;

int initialise_gpio(void)
{
	int ret;
	ret = gpio_pin_configure_dt(&dirpin0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&dirpin1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	return 0;
	
}
int initialise_i2c(void)
{

	/* Get the binding of the I2C driver  */
	dev_i2c0 = device_get_binding(I2C0);
	if (dev_i2c0 == NULL) {
	printk("Could not find  %s!\n\r",I2C0);
	return -1;
	}

    dev_i2c1 = device_get_binding(I2C1);
	if (dev_i2c1 == NULL) {
	printk("Could not find  %s!\n\r",I2C1);
	return -1;
	}
	uint8_t speed_ctrl2 = 0x80; //bit [7]
	uint8_t speed_ctrl1 = 0x00;
	for (uint8_t id = 0; id < 2; ++id)
	{
		conf_reg_write_unlock(id);
		write_reg(MOTORPARAM1, DEF_MPR, id);
		conf_reg_write_unlock(id);
		write_reg(MOTORPARAM2, DEF_BEMF, id);
		write_reg(SPEEDCTRL2, speed_ctrl2, id);
		write_reg(SPEEDCTRL1, speed_ctrl1, id);
	}
	return 0;
}
int set_motor_speed_dir(const uint16_t speed, const uint8_t dir, const uint8_t id)
{
	uint8_t speed_ctrl1 = speed & 0xFF; //bits [7:0]
	uint8_t speed_ctrl2 = (speed >> 8) | 0x80; //bit [7] override and bit [0] ([8] of SPEEDCTRL2, SPEEDCTRL1 registry pair)

	int ret;
	if (id == 0){ //first motor
		ret = gpio_pin_set_dt(&dirpin0, dir); //set direction pin
		if (ret < 0) {
			return ret;
		}
	}
	if (id == 1){ //second motor
		ret = gpio_pin_set_dt(&dirpin1, dir);
		if (ret < 0) {
			return ret;
		}
	}

	write_reg(SPEEDCTRL2, speed_ctrl2, id); //SpeedCtrl2 has to be written before SpeedCtrl1
	write_reg(SPEEDCTRL1, speed_ctrl1, id);

	return 0;
}
int write_reg(const uint8_t addr, const uint8_t val, const uint8_t id)
{
	int ret = 0;
	uint8_t reg[2] = {addr, val};
	if (driver_verbosity > 1) {printk("Writing I2C device at adress %x registry %x value %x\n", DRV10983_ADDRESS, reg[0], reg[1]);};
	if (id == 0){ //first motor
		ret = i2c_write(dev_i2c0, reg, sizeof(reg), DRV10983_ADDRESS);
	}
	if (id == 1){ //second motor
		ret = i2c_write(dev_i2c1, reg, sizeof(reg), DRV10983_ADDRESS);
	}
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", DRV10983_ADDRESS, reg[0]);
		return ret;
	}
	return 0;
}

int read_reg(const uint8_t reg_addr, const uint8_t id, uint8_t *reg_value_ptr)
{
	int ret = 0;
	uint8_t registry[2];
	registry[0] = reg_addr;
	registry[1] = 0;
	if (driver_verbosity > 3) {printk("Reading I2C device at adress %x registry %x\n", DRV10983_ADDRESS, reg_addr);};
	if (id == 0){ //first motor
		i2c_write_read(dev_i2c0, DRV10983_ADDRESS, &registry[0], 1, &registry[1], 1);
		//i2c_write_read(dev_i2c,ADXL345_I2C_ADDRESS,&data_format[0],1,&data_format[1],1);
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \n", DRV10983_ADDRESS, reg_addr);
			return ret;
		}
	}
	if (id == 1){ //first motor
		i2c_write_read(dev_i2c1, DRV10983_ADDRESS, &registry[0], 1, &registry[1], 1);
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \n", DRV10983_ADDRESS, reg_addr);
			return ret;
		}
	}
	*reg_value_ptr = registry[1];
	if (driver_verbosity > 3) {printk("Registry %x value %x\n", reg_addr, *reg_value_ptr);};
	return 0;
	/*static inline int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read)
	dev – Pointer to the device structure for an I2C controller driver configured in controller mode.
	addr – Address of the I2C device
	write_buf – Pointer to the data to be written
	num_write – Number of bytes to write
	read_buf – Pointer to storage for read data
	num_read – Number of bytes to read*/
}

int conf_reg_write_unlock(const uint8_t id)
{
	uint8_t sidata_addr = 0x03;
	uint8_t sidata_val = 0x40;
	int ret = 0;
	if (driver_verbosity > 1) {printk("Unlocking configuration register\n");};
	ret = write_reg(sidata_addr, sidata_val, id);
	return ret;
}

uint8_t was_locked[2] = {0, 0}; //store info about locked moteor until user reset

void probe_motor_lock(const uint8_t drive_id)
{
	uint8_t status_addr = 0x10;
    uint8_t status_val = 0;
	read_reg(status_addr, drive_id, &status_val);
	uint8_t is_locked = ((status_val & 0b00010000) >> 4);
	if ((was_locked[drive_id] == 0) && (is_locked > 0)){ //read only once, wait for user reset
		printk("Driver id = %u, LOCKED,\n", drive_id);
		//check_status_reg(drive_id);
		check_fault_reg(drive_id);	
	}
	if (is_locked)
		was_locked[drive_id] = 1;
}

void reset_motor_lock_flag_and_info(const uint8_t drive_id)
{
	was_locked[drive_id] = 0;
	check_fault_reg(drive_id);
}

void check_status_reg(const uint8_t drive_id)
{
    uint8_t status_addr = 0x10;
    uint8_t status_val = 0;
	read_reg(status_addr, drive_id, &status_val);
	encode_send_reg (status_addr, status_val, drive_id);
	if (driver_verbosity > 0)
	{
		if ((status_val & 0b10000000) == 0b10000000)
			printk("Driver id = %u, Over Temperature,\n", drive_id);
		if ((status_val & 0b01000000) == 0b01000000)
			printk("Driver id = %u, Sleep/Standby,\n", drive_id);
		if ((status_val & 0b00100000) == 0b00100000)
			printk("Driver id = %u, Over Current,\n", drive_id);
		if ((status_val & 0b00010000) == 0b00010000)
			printk("Driver id = %u, Motor Locked,\n", drive_id);
	}
 }

void check_fault_reg(const uint8_t drive_id)
 {
    uint8_t fault_addr = 0x1E;
    uint8_t fault_val = 0;

	read_reg(fault_addr, drive_id, &fault_val);
	encode_send_reg (fault_addr, fault_val, drive_id);
	if (driver_verbosity > 0)
	{
		if ((fault_addr & 0b00100000) == 0b00100000)
			printk("Driver id = %u, Stuck in closed loop,\n", drive_id);
		if ((fault_addr & 0b00010000) == 0b00010000)
			printk("Driver id = %u, Stuck in open loop,\n", drive_id);
		if ((fault_addr & 0b00010000) == 0b00010000)
			printk("Driver id = %u, No motor,\n", drive_id);
		if ((fault_addr & 0b00000100) == 0b00000100)
			printk("Driver id = %u, Kt abnormal,\n", drive_id);
		if ((fault_addr & 0b00000010) == 0b00000010)
			printk("Driver id = %u, Speed abnormal,\n", drive_id);
		if ((fault_addr & 0b00000001) == 0b00000001)
			printk("Driver id = %u, Lock detection current limit,\n", drive_id);
	}
	
 }

 void read_motor_speed(const uint8_t drive_id, uint8_t *speed1_val_ptr, uint8_t *speed2_val_ptr)
 {
	read_reg(MOTORSPEED1, drive_id, speed1_val_ptr); //MotorSpeed1 have to bead first
	read_reg(MOTORSPEED2, drive_id, speed2_val_ptr);
	uint16_t speed = (uint16_t)*speed1_val_ptr << 8 | *speed2_val_ptr;
	if (driver_verbosity > 1)
	{
		printk("Motor id = %u, Speed = %u,\n", drive_id, speed);
	}
	
 }

void set_uart_driver_verbosity(const int level)
 {
	driver_verbosity = level;
 }

void read_and_send_all_regs(const uint8_t drive_id)
 {
	for (uint8_t reg_addr = 0; reg_addr <= 0x2B; reg_addr++) //registry 0x00-0x2B
	{   
		uint8_t reg_val;
		read_reg(reg_addr, drive_id, &reg_val); 
		encode_send_reg(reg_addr, reg_val, drive_id); //send via bluetooth
		printk("Drive: %d, Registry: %x, Value: %x\n", drive_id, reg_addr, reg_val);
	}
 }