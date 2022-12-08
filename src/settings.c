/* SPDX-License-Identifier: Apache-2.0                              *
 * Copyright (c) 2022 Adrian Przekwas <adrian.v.przekwas@gmail.com> */

#include <zephyr.h>
#include <sys/reboot.h>
#include <device.h>
#include <string.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>

#include "settings.h"
#include "bldc_driver.h"
#include "command_decoder.h"

#define STORAGE_NODE_LABEL storage
#define DRIVE0_ID 0
#define REGS_COUNT 12

static struct nvs_fs fs;
static uint8_t drive_regs[REGS_COUNT][2];

void initialise_settings(void)
{
    int ret = 0;
	struct flash_pages_info info;

	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at FLASH_AREA_OFFSET(storage)
	 */
	fs.flash_device = FLASH_AREA_DEVICE(STORAGE_NODE_LABEL);
	if (!device_is_ready(fs.flash_device)) {
		printk("Flash device %s is not ready\n", fs.flash_device->name);
		return;
	}
	fs.offset = FLASH_AREA_OFFSET(storage);
	ret = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (ret) {
		printk("Unable to get page info\n");
		return;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	ret = nvs_mount(&fs);
	if (ret) {
		printk("Flash Init failed\n");
		return;
	}

}
void read_drive_settings(uint8_t drive_id, bool copy_to_drv)
{
	//int write_reg(const uint8_t addr, const uint8_t val, const uint8_t id)
	int ret = 0;
	ret = nvs_read(&fs, drive_id, &drive_regs, sizeof(drive_regs));
        if (ret > 0) 
		{ /* item was found, show it */
            printk("Drive: %d, Registry values:\n", drive_id);
            for (int n = 0; n < REGS_COUNT; n++) 
			{
				uint8_t reg_addr = 0x20 + n;
				uint8_t reg_val = drive_regs[n][drive_id];
				printk("Drive: %d, Registry: %x, Value: %x\n", drive_id, reg_addr, reg_val);
				if (copy_to_drv){
					write_reg(reg_addr, reg_val, drive_id);
				}
				encode_send_flash_reg(reg_addr, reg_val, drive_id);
            }
        }
}
void write_drive_settings(uint8_t drive_id)
{
    for (int n = 0; n < REGS_COUNT; n++) //registry 0x20-0x2B
    {   
        uint8_t reg_addr = 0x20 + n;
        read_reg(reg_addr, drive_id, &drive_regs[n][drive_id]); 
        printk("Drive: %d, Registry: %x, Value: %x\n", drive_id, reg_addr, drive_regs[n][drive_id]);
    }

    (void)nvs_write(&fs, drive_id, &drive_regs, sizeof(drive_regs));
}