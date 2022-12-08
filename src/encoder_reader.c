/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <sys/printk.h>

#include "encoder_reader.h"

#define SLEEP_TIME_MS  100

#define E0P0_NODE    DT_ALIAS(enc0p0)
//#define E0P0_NODE    DT_NODELABEL(enc0p0)

#if DT_NODE_HAS_STATUS(E0P0_NODE, okay)
#define E0P0_GPIO_LABEL  DT_GPIO_LABEL(E0P0_NODE, gpios)
#define E0P0_GPIO_PIN    DT_GPIO_PIN(E0P0_NODE, gpios)
#define E0P0_GPIO_FLAGS  DT_GPIO_FLAGS(E0P0_NODE, gpios)
#else
#error "Unsupported board: enc0p0 devicetree alias is not defined"
#define E0P0_GPIO_LABEL  ""
#define E0P0_GPIO_PIN    0
#define E0P0_GPIO_FLAGS  0
#endif

#define E0P1_NODE    DT_ALIAS(enc0p1)

#if DT_NODE_HAS_STATUS(E0P1_NODE, okay)
#define E0P1_GPIO_LABEL  DT_GPIO_LABEL(E0P1_NODE, gpios)
#define E0P1_GPIO_PIN    DT_GPIO_PIN(E0P1_NODE, gpios)
#define E0P1_GPIO_FLAGS  DT_GPIO_FLAGS(E0P1_NODE, gpios)
#else
#error "Unsupported board: enc0p1 devicetree alias is not defined"
#define E0P1_GPIO_LABEL  ""
#define E0P1_GPIO_PIN    0
#define E0P1_GPIO_FLAGS  0
#endif

#define E1P0_NODE    DT_ALIAS(enc1p0)

#if DT_NODE_HAS_STATUS(E1P0_NODE, okay)
#define E1P0_GPIO_LABEL  DT_GPIO_LABEL(E1P0_NODE, gpios)
#define E1P0_GPIO_PIN    DT_GPIO_PIN(E1P0_NODE, gpios)
#define E1P0_GPIO_FLAGS  DT_GPIO_FLAGS(E1P0_NODE, gpios)
#else
#error "Unsupported board: enc1p0 devicetree alias is not defined"
#define E1P0_GPIO_LABEL  ""
#define E1P0_GPIO_PIN    0
#define E1P0_GPIO_FLAGS  0
#endif

#define E1P1_NODE    DT_ALIAS(enc1p1)

#if DT_NODE_HAS_STATUS(E1P1_NODE, okay)
#define E1P1_GPIO_LABEL  DT_GPIO_LABEL(E1P1_NODE, gpios)
#define E1P1_GPIO_PIN    DT_GPIO_PIN(E1P1_NODE, gpios)
#define E1P1_GPIO_FLAGS  DT_GPIO_FLAGS(E1P1_NODE, gpios)
#else
#error "Unsupported board: enc1p1 devicetree alias is not defined"
#define E1P1_GPIO_LABEL  ""
#define E1P1_GPIO_PIN    0
#define E1P1_GPIO_FLAGS  0
#endif

#define FG0_NODE    DT_ALIAS(fg0p)

#if DT_NODE_HAS_STATUS(FG0_NODE, okay)
#define FG0_GPIO_LABEL  DT_GPIO_LABEL(FG0_NODE, gpios)
#define FG0_GPIO_PIN    DT_GPIO_PIN(FG0_NODE, gpios)
#define FG0_GPIO_FLAGS  DT_GPIO_FLAGS(FG0_NODE, gpios)
#else
#error "Unsupported board: fg0p devicetree alias is not defined"
#define FG0_GPIO_LABEL  ""
#define FG0_GPIO_PIN    0
#define FG0_GPIO_FLAGS  0
#endif

#define FG1_NODE    DT_ALIAS(fg1p)

#if DT_NODE_HAS_STATUS(FG1_NODE, okay)
#define FG1_GPIO_LABEL  DT_GPIO_LABEL(FG1_NODE, gpios)
#define FG1_GPIO_PIN    DT_GPIO_PIN(FG1_NODE, gpios)
#define FG1_GPIO_FLAGS  DT_GPIO_FLAGS(FG1_NODE, gpios)
#else
#error "Unsupported board: fg1p devicetree alias is not defined"
#define FG1_GPIO_LABEL  ""
#define FG1_GPIO_PIN    0
#define FG1_GPIO_FLAGS  0
#endif

static int64_t step_count0 = 0;
static int8_t enc0_add = 1;

static int64_t step_count1 = 0;
static int8_t enc1_add = 1;

static int64_t fg0_count = 0;
static int8_t fg0_dir = 1;

static int64_t fg1_count = 0;
static int8_t fg1_dir = 1;

static void enc0_moved_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	step_count0 += enc0_add;
}
static void enc0_moved_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool b_phase = gpio_pin_get(dev, E0P1_GPIO_PIN); //Read the status of pin
	enc0_add = b_phase ? 1 : -1;
}

static void enc1_moved_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	step_count1 += enc1_add;
}
static void enc1_moved_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool b_phase = gpio_pin_get(dev, E1P1_GPIO_PIN); //Read the status of pin
	enc1_add = b_phase ? 1 : -1;
}


static void fg0_moved(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	fg0_count += fg0_dir; 
}

static void fg1_moved(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	fg1_count += fg1_dir; 
}

/*  Define a variable of type static struct gpio_callback */
static struct gpio_callback enc0_cb_data_a;
static struct gpio_callback enc0_cb_data_b;

static struct gpio_callback enc1_cb_data_a;
static struct gpio_callback enc1_cb_data_b;

static struct gpio_callback fg0_cb_data;

static struct gpio_callback fg1_cb_data;

void initialise_encoder(void)
{
	const struct device *dev1;
	const struct device *dev2;
	int ret;

	dev1 = device_get_binding(E0P1_GPIO_LABEL);
	if (dev1 == NULL) {
		return;
	}
	dev2 = device_get_binding(E1P0_GPIO_LABEL); //E1P0 on different bus, set an unique dev for every pin?
	if (dev2 == NULL) {
		return;
	}

    ret = gpio_pin_configure(dev1, E0P0_GPIO_PIN, GPIO_INPUT | E0P0_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_configure(dev1, E0P1_GPIO_PIN, GPIO_INPUT | E0P1_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }

	ret = gpio_pin_configure(dev2, E1P0_GPIO_PIN, GPIO_INPUT | E1P0_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }

    ret = gpio_pin_configure(dev1, E1P1_GPIO_PIN, GPIO_INPUT | E1P1_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }

	/*  Configure the interrupt on the pin by calling the function gpio_pin_interrupt_configure()  */
	ret = gpio_pin_interrupt_configure(dev1, E0P0_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE); 
	if (ret < 0) {
        return;
    }
	ret = gpio_pin_interrupt_configure(dev1, E0P1_GPIO_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
        return;
    }

	ret = gpio_pin_interrupt_configure(dev2, E1P0_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE); 
	if (ret < 0) {
        return;
    }
	ret = gpio_pin_interrupt_configure(dev1, E1P1_GPIO_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
        return;
    }

	/* Initialize the static struct gpio_callback variable   */
	gpio_init_callback(&enc0_cb_data_a, enc0_moved_a, BIT(E0P0_GPIO_PIN));
	gpio_init_callback(&enc0_cb_data_b, enc0_moved_b, BIT(E0P1_GPIO_PIN));

	gpio_init_callback(&enc1_cb_data_a, enc1_moved_a, BIT(E1P0_GPIO_PIN));
	gpio_init_callback(&enc1_cb_data_b, enc1_moved_b, BIT(E1P1_GPIO_PIN)); 
	/* Add the callback function by calling gpio_add_callback()   */
	gpio_add_callback(dev1, &enc0_cb_data_a);
	gpio_add_callback(dev1, &enc0_cb_data_b);

	gpio_add_callback(dev2, &enc1_cb_data_a);
	gpio_add_callback(dev1, &enc1_cb_data_b);


	/*FG pin callback configuration*/
    ret = gpio_pin_configure(dev1, FG0_GPIO_PIN, GPIO_INPUT | FG0_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }
	ret = gpio_pin_interrupt_configure(dev1, FG0_GPIO_PIN, GPIO_INT_EDGE_RISING);
	if (ret < 0) {
        return;
    }
	gpio_init_callback(&fg0_cb_data, fg0_moved, BIT(FG0_GPIO_PIN)); 
	gpio_add_callback(dev1, &fg0_cb_data);

	/*second motor*/
	ret = gpio_pin_configure(dev1, FG1_GPIO_PIN, GPIO_INPUT | FG1_GPIO_FLAGS);
    if (ret < 0) {
        return;
    }
	ret = gpio_pin_interrupt_configure(dev1, FG1_GPIO_PIN, GPIO_INT_EDGE_RISING);
	if (ret < 0) {
        return;
    }
	gpio_init_callback(&fg1_cb_data, fg1_moved, BIT(FG1_GPIO_PIN)); 
	gpio_add_callback(dev1, &fg1_cb_data);
}

void get_enc0_val(int64_t *steps) 
{
	*steps = step_count0;
}

void get_enc1_val(int64_t *steps) 
{
	*steps = step_count1;
}

void get_fg0_val(int64_t *steps) 
{
	*steps = fg0_count;
}

void get_fg1_val(int64_t *steps) 
{
	*steps = fg1_count;
}

void set_fg_dir(uint8_t fg_id, uint8_t dir)
{
	if (fg_id == 0){
		if (dir > 0){
			fg0_dir = 1;
		}
		else {
			fg0_dir = -1;
		}
	}
	if (fg_id == 1){
		if (dir > 0){
			fg1_dir = 1;
		}
		else {
			fg1_dir = -1;
		}
	}
}