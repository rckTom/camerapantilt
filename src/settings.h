#ifndef CAMPANTILT__SETTINGS__H
#define CAMPANTILT__SETTINGS__H

#include "grbl.h"

struct SettingData {
	struct Position pos;
};

int setting_get(uint8_t reg_num, struct SettingData *data);
int setting_set(uint8_t reg_num, struct SettingData *data);
void setting_init();

#endif