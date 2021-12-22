#include "grbl.h"
#include "zephyr.h"
#include <string.h>
#include <logging/log.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(grbl, CONFIG_LOG_DEFAULT_LEVEL);

RING_BUF_DECLARE(grbl_tx_buf, 512);
RING_BUF_DECLARE(grbl_rx_buf, 512);

K_FIFO_DEFINE(grbl_cmd_fifo);
K_FIFO_DEFINE(grbl_receive_fifo);

K_CONDVAR_DEFINE(grbl_new_response_condvar);
K_MUTEX_DEFINE(grbl_cmd_mutex);

K_THREAD_STACK_DEFINE(grbl_receive_stack, 2048);

/* This timer sends a regular status report realtime command to grbl */
static void grbl_report_timer_expr(struct k_timer *dummy);
K_TIMER_DEFINE(report_timer, grbl_report_timer_expr, NULL);

static const struct device *grbl_dev;
struct k_thread grbl_receive_thread_data;

struct WorkOffset {
	int wco_num;
	struct Position offset;
};

struct GrblState currentState = { 0 };

static struct WorkOffset workOffsets[5] = { 0 };

enum grbl_message {
	GRBL_OK,
	GRBL_REPORT,
	GRBL_ERROR,
	GRBL_ALARM,
	GRBL_FEEDBACK,
	GRBL_SETTINGS,
	GRBL_STARTUP_EXEC,
	GRBL_WELCOME,
	GRBL_INVALID
};

static const char *const msg_prefix[] = {
	[GRBL_OK] = "ok",	   [GRBL_REPORT] = "<",
	[GRBL_ERROR] = "error:",   [GRBL_ALARM] = "Alarm:",
	[GRBL_FEEDBACK] = "[",	   [GRBL_SETTINGS] = "$",
	[GRBL_STARTUP_EXEC] = ">", [GRBL_WELCOME] = "Grbl"
};

static const char *const state_str[] = {
	[GRBL_STATE_IDLE] = "Idle",   [GRBL_STATE_RUN] = "Run",
	[GRBL_STATE_HOLD] = "Hold",   [GRBL_STATE_JOG] = "Jog",
	[GRBL_STATE_ALARM] = "Alarm", [GRBL_STATE_DOOR] = "Door",
	[GRBL_STATE_CHECK] = "Check", [GRBL_STATE_HOME] = "Home",
	[GRBL_STATE_SLEEP] = "Sleep"
};

static void grbl_uart_irq_tx(const struct device *dev)
{
	uint8_t *data_start;
	uint32_t bsend = 0;
	uint32_t bsize = ring_buf_get_claim(&grbl_tx_buf, &data_start, 512);

	if (bsize == 0) {
		uart_irq_tx_disable(dev);
	} else {
		bsend = uart_fifo_fill(dev, data_start, bsize);
	}
	ring_buf_get_finish(&grbl_tx_buf, bsend);
}

static void parse_position(const char *msg, float *x, float *y, float *z)
{
	char *endPtr;
	*x = strtof(msg, &endPtr);
	*y = strtof(endPtr + 1, &endPtr);
	*z = strtof(endPtr + 1, NULL);
}

static void grbl_uart_irq_rx(const struct device *dev)
{
	uint8_t *buffer;
	static char line_buffer[128] = { 0 };
	static uint8_t line_buffer_pos = 0;

	uint32_t bsize = ring_buf_put_claim(&grbl_rx_buf, &buffer, 512);
	int rsize = uart_fifo_read(dev, buffer, bsize);
	ring_buf_put_finish(&grbl_rx_buf, rsize);

	uint32_t numreceived = ring_buf_get_claim(&grbl_rx_buf, &buffer, 512);

	int i = 0;
	for (i = 0; i < numreceived; i++) {
		line_buffer[line_buffer_pos] = buffer[i];
		line_buffer_pos++;

		if (line_buffer_pos >= ARRAY_SIZE(line_buffer)) {
			line_buffer_pos = 0;
		}

		if (buffer[i] == '\n') {
			char *line = k_malloc(line_buffer_pos);

			if (line == NULL) {
				LOG_ERR("unable to allocate memory");
			} else {
				memcpy(line, line_buffer,
				       line_buffer_pos -
					       2); // remove trailing newline
				memset(line_buffer, 0, ARRAY_SIZE(line_buffer));
				line[line_buffer_pos - 2] = 0;
				line_buffer_pos = 0;

				k_fifo_put(&grbl_receive_fifo, line);
			}
		}
	}

	ring_buf_get_finish(&grbl_rx_buf, numreceived);
}

void grbl_uart_callback(const struct device *dev, void *user_data)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			grbl_uart_irq_rx(dev);
		}

		if (uart_irq_tx_ready(dev)) {
			grbl_uart_irq_tx(dev);
		}
	}
}

static enum grbl_message detect_response_type(const char *response)
{
	uint32_t response_len = strlen(response);

	for (int i = 0; i < ARRAY_SIZE(msg_prefix); i++) {
		int prefix_len = strlen(msg_prefix[i]);

		if (prefix_len > response_len) {
			prefix_len = response_len;
		}

		if (strncmp(response, msg_prefix[i], prefix_len) == 0) {
			return (enum grbl_message)i;
		}
	}

	return GRBL_INVALID;
}

static enum GrblCtrlState parse_ctrl_state(const char *state)
{
	size_t response_len = strlen(state);
	for (int i = 0; i < ARRAY_SIZE(state_str); i++) {
		int prefix_len = strlen(msg_prefix[i]);

		if (prefix_len > response_len) {
			prefix_len = response_len;
		}

		if (strncmp(state, msg_prefix[i], prefix_len) == 0) {
			return (enum GrblCtrlState)i;
		}
	}

	return 0;
}

const char *grbl_state_to_str(enum GrblCtrlState state)
{
	return state_str[(int)state];
}

static void parse_work_offsets(const char *msg)
{
	char prefix[6] = { 0 };
	for (int i = 0; i < 5; i++) {
		snprintk(prefix, ARRAY_SIZE(prefix), "[G5%d:", i + 4);

		if (strncmp(msg, prefix, 5) != 0) {
			continue;
		}

		parse_position(msg + 5, &workOffsets[i].offset.x,
			       &workOffsets[i].offset.y,
			       &workOffsets[i].offset.z);
	}
}

static void parse_report(const char *msg)
{
	char *seperator;
	char *data_start = strchr(msg, '|');

	if (data_start == NULL) {
		return;
	}

	data_start++;

	currentState.state = parse_ctrl_state(msg + 1);

	while (true) {
		seperator = strchr(data_start, ':');

		if (seperator == NULL) {
			return;
		}

		if (strncmp(data_start, "MPos", 4) == 0) {
			parse_position(seperator + 1, &currentState.pos_act.x,
				       &currentState.pos_act.y,
				       &currentState.pos_act.z);
		}

		data_start = strchr(seperator + 1, '|');

		if (data_start == NULL) {
			return;
		}
	}
}

static void grbl_receive_worker()
{
	while (true) {
		char *msg = k_fifo_get(&grbl_receive_fifo, K_FOREVER);
		enum grbl_message resp_type = detect_response_type(msg);
		// LOG_INF("%s", msg);
		switch (resp_type) {
		case GRBL_OK:
		case GRBL_ERROR:
			k_mutex_lock(&grbl_cmd_mutex, K_FOREVER);
			k_condvar_signal(&grbl_new_response_condvar);
			k_mutex_unlock(&grbl_cmd_mutex);
			break;
		case GRBL_WELCOME:
			break;
		case GRBL_ALARM:
			break;
		case GRBL_REPORT:
			parse_report(msg);
			break;
		case GRBL_SETTINGS:
			break;
		case GRBL_STARTUP_EXEC:
			break;
		case GRBL_FEEDBACK:
			parse_work_offsets(msg);
			break;
		case GRBL_INVALID:
		default:

			break;
		}

		k_free(msg);
	}
}

int grbl_send_command(const char *msg)
{
	LOG_INF("%s", msg);
	k_mutex_lock(&grbl_cmd_mutex, K_FOREVER);
	ring_buf_put(&grbl_tx_buf, msg, strlen(msg));
	uart_irq_tx_enable(grbl_dev);
	k_condvar_wait(&grbl_new_response_condvar, &grbl_cmd_mutex, K_FOREVER);
	k_mutex_unlock(&grbl_cmd_mutex);
	return 0;
}

int grbl_send_byte_no_ack(uint8_t payload)
{
	ring_buf_put(&grbl_tx_buf, &payload, 1);
	uart_irq_tx_enable(grbl_dev);
	return 0;
}

struct GrblState grbl_get_state()
{
	return currentState;
}

static void grbl_report_timer_expr(struct k_timer *dummy)
{
	grbl_send_byte_no_ack('?');
}

int grbl_initialize(const struct device *uart)
{
	grbl_dev = uart;

	uart_irq_callback_set(grbl_dev, grbl_uart_callback);
	uart_irq_rx_enable(grbl_dev);

	k_thread_create(&grbl_receive_thread_data, grbl_receive_stack,
			K_THREAD_STACK_SIZEOF(grbl_receive_stack),
			grbl_receive_worker, NULL, NULL, NULL, -1, 0,
			K_NO_WAIT);

	k_timer_start(&report_timer, K_NO_WAIT, K_SECONDS(1));
	return 0;
}
