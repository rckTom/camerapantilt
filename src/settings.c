#include "settings.h"
#include "logging/log.h"
#include <zephyr.h>
#include <stdlib.h>
#include "grbl.h"
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>
#include <device.h>

static struct nvs_fs fs;

#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(storage)
#define FLASH_NODE DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE)

LOG_MODULE_REGISTER(cam_settings, CONFIG_LOG_DEFAULT_LEVEL);

static uint8_t flash_page[4096] = { 0xFF };
static struct SettingData settings[9] = { 0 };
static struct SettingData current_setting;

// static int cam_memory_handle_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
// {
//     int rc;
//     LOG_INF("memory handle set");

//     if (len != sizeof(struct SettingData)) {
//         LOG_ERR("setting wrong size");
//         return -EINVAL;
//     }

//     rc = read_cb(cb_arg, &current_setting, sizeof(struct SettingData));
//     LOG_INF("read setting: %f %f %f", current_setting.pos.x, current_setting.pos.y, current_setting.pos.z);

//     if (rc > 0) {
//         return 0;
//     }

//     LOG_ERR("unable to read");
//     return rc;
// }

// struct settings_handler cam_memory_handler = {
//     .name = "cam",
//     .h_set = cam_memory_handle_set,
// };

// static int get_key(uint8_t reg_num, char *key, size_t size)
// {
//     if (reg_num < 0 || reg_num > 9){
//         return -EINVAL;
//     }

//     snprintk(key, size, "cam/%d", reg_num);
//     return 0;
// }

int setting_get(uint8_t reg_num, struct SettingData *data)
{
	int rc;
	LOG_INF("load setting");
	char *cmd[64];

	if (reg_num >= 0 && reg_num < 6) {
		snprintk(cmd, ARRAY_SIZE(cmd), "G5%d\n", reg_num + 4);
		grbl_send_command(cmd);
		grbl_send_command("G0 X0 Y0\n");
	} else if (reg_num == 6) {
		grbl_send_command("G28\n");
	} else if (reg_num == 7) {
		grbl_send_command("G30\n");
	}

	if (rc < 0) {
		LOG_ERR("unable to read setting. rc: %d", rc);
		return rc;
	}

	memcpy(data, &current_setting, sizeof(struct SettingData));
	return 0;
}

int setting_set(uint8_t reg_num, struct SettingData *data)
{
	char *cmd[64];

	int rc = 0;

	if (reg_num >= 0 && reg_num < 6) {
		snprintk(cmd, ARRAY_SIZE(cmd), "G10 L2 P%d X%f Y%f\n",
			 reg_num + 1, data->pos.x, data->pos.y);
		grbl_send_command(cmd);
	} else if (reg_num == 6) {
		grbl_send_command("G28.1\n");
	} else if (reg_num == 7) {
		grbl_send_command("G30.1\n");
	}

	return rc;
}

void setting_init()
{
	int rc = 0;
	struct flash_pages_info info;
	const struct device *flash_dev;

	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at FLASH_AREA_OFFSET(storage)
	 */
	flash_dev = DEVICE_DT_GET(FLASH_NODE);
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash device %s is not ready\n", flash_dev->name);
		return;
	}
	fs.offset = FLASH_AREA_OFFSET(storage);
	rc = flash_get_page_info_by_offs(flash_dev, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return;
	}
	fs.sector_size = info.size;
	fs.sector_count = 8U;

	//rc = nvs_init(&fs, flash_dev->name);
	//if (rc) {
	//	LOG_ERR("Flash Init failed\n");
	//	return;
	//}
}