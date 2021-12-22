#include <stdint.h>

enum visca_commands {
	PTD_UP,
	PTD_DOWN,
	PTD_LEFT,
	PTD_RIGHT,
	PTD_UPLEFT,
	PTD_UPRIGHT,
	PTD_DOWNLEFT,
	PTD_DOWNRIGHT,
	PTD_STOP,
	PTD_ABS,
	PTD_REL,
	PTD_HOME,
	PTD_RESET,
	CAM_MEMORY_SET,
	CAM_MEMORY_RECALL
};

struct visca_packet_raw {
	uint8_t addr;
	uint8_t length;
	uint8_t data[14];
};

struct visca_ptd_jog_motion {
	uint8_t pan_speed;
	uint8_t titlt_speed;
} __attribute__((packed));

struct visca_ptd_abs_rel_motion {
	uint8_t pan_speed;
	uint8_t titlt_speed;
	uint32_t pan_pos;
	uint32_t tilt_pos;
} __attribute__((packed));

struct visca_cam_memory {
	uint8_t memory_slot;
} __attribute__((packed));

struct visca_command {
	enum visca_commands cmd;
	union {
		struct visca_ptd_jog_motion ptd_jog_motion;
		struct visca_ptd_abs_rel_motion ptd_abs_motion;
		struct visca_ptd_abs_rel_motion ptd_rel_motion;
		struct visca_cam_memory cam_memory;
	} payload;
};

int visca_raw_packet_to_command(struct visca_packet_raw *raw_packet,
				struct visca_command *cmd);