#ifndef CAMPANTILT__GRBL__H
#define CAMPANTILT__GRBL__H

#include <stdint.h>
#include <device.h>

struct Position {
	float x, y, z;
};

enum GrblCtrlState {
	GRBL_STATE_IDLE,
	GRBL_STATE_RUN,
	GRBL_STATE_HOLD,
	GRBL_STATE_JOG,
	GRBL_STATE_ALARM,
	GRBL_STATE_DOOR,
	GRBL_STATE_CHECK,
	GRBL_STATE_HOME,
	GRBL_STATE_SLEEP
};

struct GrblState {
	char state;
	struct Position pos_act;
};

int grbl_send_command(const char *msg);
int grbl_send_byte_no_ack(uint8_t payload);
int grbl_initialize(const struct device *uart);
struct GrblState grbl_get_state();

#endif