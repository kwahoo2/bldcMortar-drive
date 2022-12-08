/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#include "command_decoder.h"
#include "bldc_driver.h"
#include "ble_comm.h"
#include "encoder_reader.h"
#include "settings.h"

void decode_and_run (uint64_t *command)
{
    /*
    First 3 bytes descibe command type, 4 byte is drive id (0-255), later 4 data value, eg. SPF0wwww (wwww ASCII=>2004318071 DEC)
    */
    char com_str[9];
    uint32_t com_val = 0;
    const uint8_t fwd_dir = 1; //setup of dir pin
    const uint8_t bwd_dir = 0;

    printk("Size %u\n", sizeof(*command));
    sprintf (com_str, "%s", (char*)command);
    printk("Decoding command %s\n", com_str);
    com_val = ((*command) >> 32) & 0xFFFFFFFF; //later 4 bytes are actually first Little Endian, struct (<l) in the Python side
    uint8_t drive_id = 0;
    drive_id = (*command >> 24) & 0xFF; 

    if (strncmp (com_str,"SPF",3) == 0){ //forward revolution
        printk("Speed command SPFx, id = %u, speed value =  %u\n", drive_id, com_val);
        uint16_t speed = com_val & 0xFFFF;
        set_motor_speed_dir(speed, fwd_dir, drive_id);
        set_fg_dir(drive_id, fwd_dir);
    }
    if (strncmp (com_str,"SPB",3) == 0){ //backward revolution
        printk("Speed command SPBx, id = %u, speed value =  %u\n", drive_id, com_val);
        uint16_t speed = com_val & 0xFFFF;
        set_motor_speed_dir(speed, bwd_dir, drive_id);
        set_fg_dir(drive_id, bwd_dir);
    }
    if (strncmp (com_str,"WRG",3) == 0){ //write registry command
        uint8_t req_reg = (com_val >> 16) & 0xFF;
        uint8_t write_val = (com_val >> 24) & 0xFF;
        if (((req_reg <= 0x03) || ((req_reg >= 0x20) && (req_reg <=0x2B)))) // do not try writing to read only registry
        {
            printk("Write registry command WRGx, id = %u, requested register = %#X, write value = %#X\n", drive_id, req_reg, write_val);
            if ((req_reg >= 0x20) && (req_reg <=0x2B))
            {
                conf_reg_write_unlock(drive_id);
            }
            write_reg(req_reg, write_val, drive_id);
        }

    }
    if (strncmp (com_str,"RRG",3) == 0){ //read registry command
        uint8_t req_reg = (com_val >> 24) & 0xFF; //last byte is required register adress
        uint8_t read_val = 0;
        printk("Read registry command RRGx, id = %u, requested register = %#X,\n", drive_id, req_reg);
        read_reg(req_reg, drive_id, &read_val);
        printk("Read registry command RRGx, id = %u, requested register = %#X, read value = %#X\n", drive_id, req_reg, read_val);
        encode_send_reg (req_reg, read_val, drive_id);
    }
    if (strncmp (com_str,"RCF",3) == 0){ //read and copy registers from flash
        printk("Read and copy from flash, id = %u\n", drive_id);
        read_drive_settings(drive_id, true); //true means, registers are not only read but also copied to the drv regs
        read_and_send_all_regs(drive_id); //send info back to PC
    }
    if (strncmp (com_str,"LOA",3) == 0){ //read registers from flash without copying
        printk("Read and copy from flash, id = %u\n", drive_id);
        read_drive_settings(drive_id, false);
        read_and_send_all_regs(drive_id);
    }
    if (strncmp (com_str,"WFL",3) == 0){ //write registers to flash
        printk("Write from flash, id = %u\n", drive_id);
        write_drive_settings(drive_id);
        read_drive_settings(drive_id, false); //send info back to PC
        
    }
    if (strncmp (com_str,"SDV",3) == 0){ //set driver verbosity  command
        uint8_t level = (com_val >> 24) & 0xFF; //last byte is required register adress
        printk("Setting driver UART verbosity, level = %u\n", level);
        set_uart_driver_verbosity(level);
    }
    if (strncmp (com_str,"RAR",3) == 0){ //read and send all registers
        read_and_send_all_regs(0); //read all drv registers
        read_and_send_all_regs(1);
        read_drive_settings(0, false); //read all flash registers without copying to the drv
        read_drive_settings(1, false);
    }
    if (strncmp (com_str,"RDR",3) == 0){ //read and send registers of a single drive
        read_and_send_all_regs(drive_id); //read all drv registers
    }
    if (strncmp (com_str,"RLF",3) == 0){
        reset_motor_lock_flag_and_info(drive_id); //reset lock flag created after status register read, re-read fault register
    }

}

void encode_send_reg(uint8_t reg_addr, uint8_t reg_value, uint8_t drive_id)
 {
    /*Python snipped to generate 3 byte command identifier
    def c(s):
        sr = s[::-1] #reversed
        nchars = len(s)
        x = sum(ord(sr[byte])<<8*(nchars-byte-1) for byte in range(nchars))
        print (s, f"0x{x:X}")

    c('VRG')*/

    /*uint64_t command = 0;
    command = command | 0x56; //V
    command = command | (0x52 << 8); //R
    command = command | (0x47 << 16); //G*/

    //VRG000xy
    uint64_t command = 0x475256;

    command = command | ((uint64_t)drive_id << 24);
    command = command | ((uint64_t)reg_addr << 48);
    command = command | ((uint64_t)reg_value << 56);  
    set_command_val(&command);
 }
 void encode_send_flash_reg(uint8_t reg_addr, uint8_t reg_value, uint8_t drive_id)
 {
    //VRF000xy
    uint64_t command = 0x465256;
    command = command | ((uint64_t)drive_id << 24);
    command = command | ((uint64_t)reg_addr << 48);
    command = command | ((uint64_t)reg_value << 56);  
    set_command_val(&command);
 }
void encode_motor_speed(uint8_t speed1, uint8_t speed2, uint8_t drive_id)
 {
    //MSP000ss
    uint64_t command = 0x50534D;
    command = command | ((uint64_t)drive_id << 24);
    command = command | ((uint64_t)speed2 << 48);
    command = command | ((uint64_t)speed1 << 56);  
    set_command_val(&command);
 }

 void encode_fg_value(int64_t fg_count, uint8_t drive_id)
 {
    uint64_t base_command = 0x46 | (0x47 << 8); //FG
    //FGRxyyyy
    uint32_t firsthalf = (uint32_t)(fg_count >> 32); //https://en.wikipedia.org/wiki/Two's_complement
    uint64_t command = base_command | (0x52 << 16); //FGR
    command = command | ((uint64_t)drive_id << 24);
    command = command | ((uint64_t)firsthalf << 32);
    set_command_val(&command);

    //FGrxyyyy
    uint32_t secondhalf = (uint32_t)(fg_count & 0xFFFFFFFF);
    command = base_command | (0x72 << 16); //FGr
    command = command | ((uint64_t)drive_id << 24);
    command = command | ((uint64_t)secondhalf << 32);
    set_command_val(&command);
 }

void encode_gps_info(uint8_t info_type, char *info)
 {
    int info_size = strlen(info);
	if (info_size > 6) {
		printk("UART GPS message too long, dropping!\n");
		return;
	}
    //Gtssssss
    uint64_t command = 0;
    memcpy(&command, info, info_size);
    command = (uint64_t)(command << 16);
    command = command | 0x47;
    command = command | ((uint64_t)info_type << 8);
    set_command_val(&command);

 }

 