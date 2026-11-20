/*
 * RoboticArm bridge firmware, step 1: talk to one CubeMars AK motor.
 * Board: Seeed Studio XIAO STM32C5 (Zephyr)
 *
 * Written in C: Seeed's PlatformIO package builds with GCC 8.2, which can't
 * compile the board's CMSIS headers as C++.
 *
 * What it does:
 *   - Starts classic CAN 2.0 at 1 Mbps with 29-bit (extended) IDs on the
 *     XIAO's onboard transceiver (CAN_H / CAN_L pads on the back).
 *   - Prints the motor's feedback: position, speed, current, temperature, error.
 *   - Optional slow test move (see TEST_MOVE below).
 *
 * Before flashing, use the R-Link + CubeMars Tool V3.0 to:
 *   1. set the motor's CAN ID to MOTOR_ID (1)
 *   2. turn on timed CAN feedback (e.g. 100 Hz)
 * Otherwise the motor stays quiet until it receives a command.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>
#include <stdint.h>

/* ---- Settings ----------------------------------------------------------- */

#define CAN_BITRATE 1000000 /* CubeMars AK V3 CAN bitrate */
#define MOTOR_ID    1       /* driver ID you set with the R-Link */

/* Set to 1 ONLY with the motor clamped to the bench and nothing attached.
 * It turns the motor +10 degrees slowly, waits 4 s, then goes back. */
#define TEST_MOVE 0

/* Servo-mode command type goes in bits 8..28 of the extended CAN ID;
 * the driver ID is bits 0..7 (CubeMars AK manual, servo mode protocol). */
#define MODE_POSITION_VELOCITY 6

/* Slow test-move limits. Speed/accel fields are in units of 10 ERPM
 * (electrical RPM = rotor RPM x 14 pole pairs; output RPM = rotor RPM / 36). */
#define TEST_SPEED 252 /* 2520 ERPM, about 5 RPM at the output */
#define TEST_ACCEL 252 /* 2520 ERPM/s */

/* ------------------------------------------------------------------------- */

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

CAN_MSGQ_DEFINE(rx_msgq, 32);

static int16_t read_be16(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void write_be16(uint8_t *p, int16_t v)
{
	uint16_t u = (uint16_t)v;

	p[0] = (uint8_t)(u >> 8);
	p[1] = (uint8_t)u;
}

static void write_be32(uint8_t *p, int32_t v)
{
	uint32_t u = (uint32_t)v;

	p[0] = (uint8_t)(u >> 24);
	p[1] = (uint8_t)(u >> 16);
	p[2] = (uint8_t)(u >> 8);
	p[3] = (uint8_t)u;
}

/* Prints a fixed-point value: print_fixed(123, 1) -> "12.3", print_fixed(-5, 2) -> "-0.05" */
static void print_fixed(int32_t value, int decimals)
{
	unsigned int mag = value < 0 ? (unsigned int)(-value) : (unsigned int)value;
	const char *sign = value < 0 ? "-" : "";

	if (decimals == 1) {
		printk("%s%u.%u", sign, mag / 10, mag % 10);
	} else {
		printk("%s%u.%u%u", sign, mag / 100, (mag / 10) % 10, mag % 10);
	}
}

/* Feedback frame layout (servo mode, 8 bytes, big-endian):
 *   0-1 position (0.1 deg)  2-3 speed (x10 ERPM)  4-5 current (0.01 A)
 *   6 temperature (deg C)   7 error code */
static void print_feedback(const struct can_frame *f)
{
	printk("id=0x%08x ", (unsigned int)f->id);
	if (f->dlc != 8) {
		printk("dlc=%u (not a feedback frame)\n", (unsigned int)f->dlc);
		return;
	}

	int16_t pos = read_be16(&f->data[0]);
	int16_t spd = read_be16(&f->data[2]);
	int16_t cur = read_be16(&f->data[4]);
	int temp = (int8_t)f->data[6];
	unsigned int err = f->data[7];

	printk("pos=");
	print_fixed(pos, 1);
	printk(" deg  spd=%d erpm  cur=", spd * 10);
	print_fixed(cur, 2);
	printk(" A  temp=%d C  err=%u\n", temp, err);
}

static int send_position_velocity(uint8_t id, int32_t pos_deg_x10000, int16_t speed, int16_t accel)
{
	struct can_frame frame = {0};

	frame.id = ((uint32_t)MODE_POSITION_VELOCITY << 8) | id;
	frame.flags = CAN_FRAME_IDE; /* extended (29-bit) ID */
	frame.dlc = 8;
	write_be32(&frame.data[0], pos_deg_x10000);
	write_be16(&frame.data[4], speed);
	write_be16(&frame.data[6], accel);
	return can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
}

int main(void)
{
	k_msleep(2000); /* give the PC a moment to open the USB serial port */
	printk("\nRoboticArm bridge: CAN test\n");

	if (!device_is_ready(can_dev)) {
		printk("CAN device not ready\n");
		return 0;
	}

	int ret = can_set_mode(can_dev, CAN_MODE_NORMAL); /* classic CAN, not FD */
	if (ret != 0) {
		printk("can_set_mode failed (%d)\n", ret);
		return 0;
	}

	ret = can_set_bitrate(can_dev, CAN_BITRATE);
	if (ret != 0) {
		printk("can_set_bitrate failed (%d)\n", ret);
		return 0;
	}

	struct can_filter filter = {0};

	filter.id = 0;
	filter.mask = 0; /* accept every extended-ID frame */
	filter.flags = CAN_FILTER_IDE;
	ret = can_add_rx_filter_msgq(can_dev, &rx_msgq, &filter);
	if (ret < 0) {
		printk("can_add_rx_filter_msgq failed (%d)\n", ret);
		return 0;
	}

	ret = can_start(can_dev);
	if (ret != 0) {
		printk("can_start failed (%d)\n", ret);
		return 0;
	}

	printk("CAN running at %u bit/s, waiting for motor %u...\n",
	       (unsigned int)CAN_BITRATE, (unsigned int)MOTOR_ID);

	bool have_baseline = false;
	int32_t baseline = 0; /* starting position, deg x10000 */
	int move_step = 0;    /* 0 = not started, 1 = moved out, 2 = back home */
	int64_t move_time = 0;
	int64_t last_print = 0;
	int64_t last_rx = k_uptime_get();

	while (true) {
		struct can_frame frame;

		if (k_msgq_get(&rx_msgq, &frame, K_MSEC(100)) == 0) {
			int64_t now = k_uptime_get();

			last_rx = now;
			if (now - last_print >= 200) { /* at most 5 lines per second */
				print_feedback(&frame);
				last_print = now;
			}
			if (!have_baseline && frame.dlc == 8 && (frame.id & 0xFF) == MOTOR_ID) {
				/* 0.1 deg -> deg x10000 */
				baseline = (int32_t)read_be16(&frame.data[0]) * 1000;
				have_baseline = true;
			}
		} else if (k_uptime_get() - last_rx > 2000) {
			printk("No CAN frames for 2 s. Check 24 V power, CAN_H/CAN_L wiring, "
			       "termination, motor ID and the feedback setting.\n");
			last_rx = k_uptime_get();
		}

		if (TEST_MOVE && have_baseline) {
			int64_t now = k_uptime_get();

			if (move_step == 0) {
				printk("Test move: +10 deg\n");
				send_position_velocity(MOTOR_ID, baseline + 10 * 10000,
						       TEST_SPEED, TEST_ACCEL);
				move_step = 1;
				move_time = now;
			} else if (move_step == 1 && now - move_time > 4000) {
				printk("Test move: back to start\n");
				send_position_velocity(MOTOR_ID, baseline, TEST_SPEED, TEST_ACCEL);
				move_step = 2;
			}
		}
	}
	return 0;
}
