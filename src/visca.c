#include "visca.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(visca, CONFIG_LOG_DEFAULT_LEVEL);

static uint16_t get_pos(struct visca_packet_raw *raw_packet, int offset)
{
	uint16_t pos = 0;

	for (int i = 0; i < 4; i++) {
		pos += raw_packet->data[offset + 3 - i] << i;
	}

	return pos;
}

int visca_raw_packet_to_command(struct visca_packet_raw *raw_packet,
				struct visca_command *cmd)
{
	/* Pan Tilt Jog Command */
	if (raw_packet->length == 7 && raw_packet->data[0] == 0x01 &&
	    raw_packet->data[1] == 0x06 && raw_packet->data[2] == 0x01) {
		/* Jog commands */
		if (raw_packet->data[2] == 1) {
			uint16_t id = (raw_packet->data[6] << 8) +
				      raw_packet->data[5];
			switch (id) {
			case 0x0101:
				cmd->cmd = PTD_UPLEFT;
				break;

			case 0x0102:
				cmd->cmd = PTD_DOWNLEFT;
				break;

			case 0x0103:
				cmd->cmd = PTD_LEFT;
				break;

			case 0x0201:
				cmd->cmd = PTD_UPRIGHT;
				break;

			case 0x0202:
				cmd->cmd = PTD_DOWNRIGHT;
				break;

			case 0x0203:
				cmd->cmd = PTD_RIGHT;
				break;

			case 0x0301:
				cmd->cmd = PTD_UP;
				break;

			case 0x0302:
				cmd->cmd = PTD_DOWN;
				break;

			case 0x0303:
				cmd->cmd = PTD_STOP;
				break;

			default:
				return -1;
				break;
			}

			cmd->payload.ptd_jog_motion.pan_speed =
				raw_packet->data[3];
			cmd->payload.ptd_jog_motion.titlt_speed =
				raw_packet->data[4];
			return 0;
		}
	}

	/* Pan Tilt Position Command */
	if (raw_packet->length == 13 &&
	    (raw_packet->data[2] == 0x02 || raw_packet->data[2] == 0x03)) {
		/* Abs Positionig command */
		if (raw_packet->data[2] == 0x02) {
			cmd->cmd = PTD_ABS;
			cmd->payload.ptd_abs_motion.pan_pos =
				get_pos(raw_packet, 5);
			cmd->payload.ptd_abs_motion.tilt_pos =
				get_pos(raw_packet, 9);
			cmd->payload.ptd_abs_motion.pan_speed =
				raw_packet->data[3];
			cmd->payload.ptd_abs_motion.titlt_speed =
				raw_packet->data[4];
			return 0;
		}

		/* Rel Positionig command */
		if (raw_packet->data[2] == 0x03) {
			cmd->cmd = PTD_REL;
			cmd->payload.ptd_rel_motion.pan_pos =
				get_pos(raw_packet, 5);
			cmd->payload.ptd_rel_motion.tilt_pos =
				get_pos(raw_packet, 9);
			cmd->payload.ptd_rel_motion.pan_speed =
				raw_packet->data[3];
			cmd->payload.ptd_rel_motion.titlt_speed =
				raw_packet->data[4];
			return 0;
		}
	}

	if (raw_packet->length == 3 && raw_packet->data[0] == 0x01 &&
	    raw_packet->data[1] == 0x06) {
		if (raw_packet->data[2] == 0x04) {
			cmd->cmd = PTD_HOME;
			return 0;
		}

		if (raw_packet->data[2] == 0x05) {
			cmd->cmd = PTD_RESET;
			return 0;
		}
	}

	if (raw_packet->length == 5 && raw_packet->data[0] == 0x01 &&
	    raw_packet->data[1] == 0x04 && raw_packet->data[2] == 0x3f &&
	    raw_packet->data[3] == 0x01) {
		cmd->cmd = CAM_MEMORY_SET;
		cmd->payload.cam_memory.memory_slot = raw_packet->data[4];
		return 0;
	}

	if (raw_packet->length == 5 && raw_packet->data[0] == 0x01 &&
	    raw_packet->data[1] == 0x04 && raw_packet->data[2] == 0x3f &&
	    raw_packet->data[3] == 0x02) {
		cmd->cmd = CAM_MEMORY_RECALL;
		cmd->payload.cam_memory.memory_slot = raw_packet->data[4];
		return 0;
	}

	return -1;
}