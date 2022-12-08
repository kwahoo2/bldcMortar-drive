# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com>

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bldcmortar-drive)

target_sources(app PRIVATE
  src/main.c
  src/ble_comm.c
  src/usb_comm.c
  src/bldc_driver.c
  src/encoder_reader.c
  src/command_decoder.c
  src/settings.c
  src/gps.c
)
