/*
 * camera pantilt controller
 *
 * Copyright (c) Thomas Schmid, 2021
 *
 * Authors:
 *  Thomas Schmid <tom@lfence.de>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.  See
 * the LICENSE file in the top-level directory.
 */

#include <zephyr.h>
#include <logging/log.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <settings/settings.h>
#include "visca.h"
#include "grbl.h"
#include "math.h"
#include "settings.h"

RING_BUF_DECLARE(visca_rxbuf, 64);

K_FIFO_DEFINE(visca_cmd_fifo);
LOG_MODULE_REGISTER(camerapantilt, CONFIG_LOG_DEFAULT_LEVEL);

enum visca_parser_state {
	WAIT_FOR_ADDR,
	READ_DATA,
	READ_ENDFRAME,
};

static enum visca_parser_state parser_state = WAIT_FOR_ADDR;
static struct visca_packet_raw received_packet;
static bool jog_active;
static int pan_speed, tilt_speed;
static float xstep, ystep;
static char cmd_buffer[128];

const struct SettingData defaultSetting = { .pos = { .x = 0, .y = 0, .z = 0 } };

struct SettingData currentSetting;

void visca_uart_irq_rx(const struct device *dev)
{
	uint8_t *buffer;
	uint32_t bsize =
		ring_buf_put_claim(&visca_rxbuf, &buffer, visca_rxbuf.size);
	int rsize = uart_fifo_read(dev, buffer, bsize);
	ring_buf_put_finish(&visca_rxbuf, rsize);
	uint32_t numreceived = ring_buf_capacity_get(&visca_rxbuf) -
			       ring_buf_space_get(&visca_rxbuf);

	for (int i = 0; i < numreceived; i++) {
		if (parser_state == WAIT_FOR_ADDR) {
			ring_buf_get(&visca_rxbuf, &received_packet.addr, 1);
			if (received_packet.addr < 0x81 ||
			    received_packet.addr > 0x88) {
				continue;
			}
			parser_state = READ_DATA;
			received_packet.length = 0;
			continue;
		}

		if (parser_state == READ_DATA) {
			uint8_t data;
			ring_buf_get(&visca_rxbuf, &data, 1);

			if (data == 0xFF) {
				parser_state = WAIT_FOR_ADDR;
				struct visca_command *visca_cmd =
					k_malloc(sizeof(struct visca_command));

				if (visca_cmd == NULL) {
					LOG_ERR("out of memory. Dropping packets");
					continue;
				}

				if (visca_raw_packet_to_command(
					    &received_packet, visca_cmd) != 0) {
					LOG_INF("unable to parse visca command");
					k_free(visca_cmd);
					continue;
				}

				if (k_fifo_alloc_put(&visca_cmd_fifo,
						     visca_cmd) != 0) {
					LOG_ERR("unable to allocate memory for fifo element");
					k_free(visca_cmd);
				}
				continue;
			}

			received_packet.length++;
			received_packet.data[received_packet.length - 1] = data;
		}
	}
}

void visca_uart_callback(const struct device *dev, void *user_data)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			visca_uart_irq_rx(dev);
		}
	}
}

void grbl_worker()
{
	bool last_jog = false;

	while (true) {
		if (last_jog != jog_active) {
			if (!jog_active) {
				grbl_send_byte_no_ack(0x85);
			}
			last_jog = jog_active;
		}

		if (jog_active) {
			char jogcmd[64];
			float speed = 0.01;

			snprintk(jogcmd, ARRAY_SIZE(jogcmd),
				 "$J=G91 X%fY%f F%f\n", -ystep * 0.01,
				 -xstep * 0.01, speed);
			grbl_send_command(jogcmd);
		} else {
			k_sleep(K_MSEC(50));
		}
	}
}

void main(void)
{
	const struct device *visca_dev = device_get_binding("UART_6");
	const struct device *grbl_dev = device_get_binding("UART_1");

	if (visca_dev == NULL) {
		LOG_ERR("unable to get uart_device");
		return;
	}

	if (grbl_dev == NULL) {
		LOG_ERR("unable to get grbl serial port");
		return;
	}

	const struct uart_config grbl_uart_config = {
		.baudrate = 115200,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};

	const struct uart_config visa_uart_config = {
		.baudrate = 9600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};

	if (uart_configure(grbl_dev, &grbl_uart_config) != 0) {
		LOG_ERR("unable to configure uart for grbl");
		return;
	}

	if (uart_configure(visca_dev, &visa_uart_config) != 0) {
		LOG_ERR("unable to configure uart for visca");
		return;
	}

	setting_init();
	memcpy(&currentSetting, &defaultSetting, sizeof(struct SettingData));

	/* Visca serial connection */
	uart_irq_callback_set(visca_dev, visca_uart_callback);
	uart_irq_rx_enable(visca_dev);

	grbl_initialize(grbl_dev);
	grbl_send_command("\r\n\r\n"); /* Wake up grbl */

	LOG_INF("start homing");
	grbl_send_command("$H\n"); /* Run a homing cycle */

	//grbl_send_command("$X\n");
	grbl_send_command("$10=1\n");
	grbl_send_command("$#\n");
	grbl_send_command("G54\n");
	grbl_send_command("G0 X0 Y0\n");

	LOG_INF("ready");
	while (1) {
		struct visca_command *cmd =
			k_fifo_get(&visca_cmd_fifo, K_FOREVER);
		LOG_INF("received visca packet");
		if (cmd == NULL) {
			goto OUT;
		}

		if (cmd->cmd == PTD_ABS || cmd->cmd == PTD_REL) {
			if (jog_active) {
				jog_active = false;
				k_sleep(K_MSEC(
					100)); // wait some time to make sure jog is aborted
			}

			if (cmd->cmd == PTD_ABS) {
				grbl_send_command(
					"G90\n"); /* asolute position mode */
			} else {
				grbl_send_command(
					"G91\n"); /* relative position mode */
			}

			snprintk(cmd_buffer, ARRAY_SIZE(cmd_buffer),
				 "G0 X%fY%f\n",
				 cmd->payload.ptd_abs_motion.pan_pos / 1000.0,
				 cmd->payload.ptd_abs_motion.tilt_pos / 1000.0);

			grbl_send_command(cmd_buffer);
			goto OUT;
		}

		/* Check if jog command */
		if (cmd->cmd == PTD_DOWN || cmd->cmd == PTD_DOWNLEFT ||
		    cmd->cmd == PTD_DOWNRIGHT || cmd->cmd == PTD_UP ||
		    cmd->cmd == PTD_UPLEFT || cmd->cmd == PTD_UPRIGHT ||
		    cmd->cmd == PTD_LEFT || cmd->cmd == PTD_RIGHT ||
		    cmd->cmd == PTD_STOP) {
			pan_speed = cmd->payload.ptd_jog_motion.pan_speed;
			tilt_speed = cmd->payload.ptd_jog_motion.titlt_speed;

			if (cmd->cmd != PTD_STOP) {
				jog_active = true;
			} else {
				jog_active = false;
				grbl_send_byte_no_ack(0x85);
			}

			switch (cmd->cmd) {
			case PTD_UP:
				xstep = 0;
				ystep = 1;
				break;
			case PTD_DOWN:
				xstep = 0;
				ystep = -1;
				break;
			case PTD_LEFT:
				xstep = -1;
				ystep = 0;
				break;
			case PTD_RIGHT:
				xstep = 1;
				ystep = 0;
				break;
			case PTD_UPLEFT:
				xstep = -1;
				ystep = 1;
				break;
			case PTD_UPRIGHT:
				xstep = 1;
				ystep = 1;
				break;
			case PTD_DOWNLEFT:
				xstep = -1;
				ystep = -1;
				break;
			case PTD_DOWNRIGHT:
				xstep = 1;
				ystep = -1;
				break;
			default:
				xstep = 0;
				ystep = 0;
				break;
			}
		}

		if (cmd->cmd == PTD_HOME) {
			grbl_send_command("$H\n");
			goto OUT;
		}

		if (cmd->cmd == PTD_RESET) {
			grbl_send_command("$X\n");
			goto OUT;
		}

		if (cmd->cmd == CAM_MEMORY_RECALL) {
			LOG_INF("memory recall");
			currentSetting.pos = grbl_get_state().pos_act;
			setting_get(cmd->payload.cam_memory.memory_slot,
				    &currentSetting);
			//grbl_send_command(cmd_buffer);
			goto OUT;
		}

		if (cmd->cmd == CAM_MEMORY_SET) {
			LOG_INF("memory set");
			currentSetting.pos = grbl_get_state().pos_act;
			setting_set(cmd->payload.cam_memory.memory_slot,
				    &currentSetting);
		}

	OUT:
		k_free(cmd);
	}
}

K_THREAD_DEFINE(grbl_worker_thread, 1024, grbl_worker, NULL, NULL, NULL, -1, 0,
		0);
