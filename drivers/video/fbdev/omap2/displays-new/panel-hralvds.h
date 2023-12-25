/*********************************************************
*
*  Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __PANEL_HRALVDS_H
#define __PANEL_HRALVDS_H

#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <video/omapdss.h>
#include "hralvds-proto.h"

/* these macros are used by all of the panel-hralvds*.c code */

#if 0
#define HRALVDS_INIT_DESERIALIZER
#endif

#if 1
#define HRALVDS_HDCP_ENABLED
#endif

#if 1
#define HRALVDS_STARTUP_DEBUG
#endif

#if 1
/* define to be able to use the dbg_lvl to produce kernel log messages */
#define HRALVDS_LOG_DEBUG
#endif

#if defined HRALVDS_LOG_DEBUG
/*
 * some debug logging settings.  these are progressive, e.g., 2 implies 1.
 * none: don't display anything regarding i2c communications.
 * event: show events received from the display (not much detail).
 * dump: show the buffers received over i2c from the display.
 */
#define HRALVDS_DBG_LEVEL_NONE	(0)
#define HRALVDS_DBG_LEVEL_EVENT	(1 << 0)
#define HRALVDS_DBG_LEVEL_DUMP	(1 << 1)
#define HRALVDS_DBG_LEVEL_ISR	(1 << 2)

#define HRALVDS_LOG_DEBUG_DEFAULT_LEVEL	(HRALVDS_DBG_LEVEL_ISR)
#endif

#define HRALVDS_COLOR_MAX	0x0d
#define HRALVDS_PATTERN_SHOW_TIME_MAX 0x1f

/* These structs are common to all hralvds display panels. */

/*
 * the radio frequency command is common to all HRA LVDS panels.
 */
struct hralvds_radio_freq_cmd_struct {
	u8 cmd_id;
	union {
		u8 data[HRALVDS_RADIO_FREQ_CMD_SIZE];
		struct {
			u8 band;
			u8 freq_thousands_hundreds;
			u8 freq_tens_ones;
			u8 freq_tenths_hundredths;
		} radio;
	};
	u8 checksum;
} __packed;

/*
 * the power command is common to all HRA LVDS panels.
 */
struct hralvds_power_cmd_struct {
	u8 cmd_id;
	union {
		u8 data[HRALVDS_POWER_CMD_SIZE];
		struct {
			u8  ctrl; /* gradual change time, day/night, lcd, led */
			u8  led_face_sw_state;
			u16 lcd_backlight_lum;
			u16 led_face_sw_lum;  /* capacitive buttons */
			u16 led_hard_key_lum; /* vol knob ring */
		} power;
	};
	u8 checksum;
} __packed;

/*
 * the video command is common to all HRA LVDS panels.
 */
struct hralvds_video_cmd_struct {
	u8 cmd_id;
	union {
		u8 data[HRALVDS_VIDEO_CMD_SIZE];
		struct {
			u8 contrast;
			u8 black_level;
			u8 color;
			u8 tint;
		} video;
	};
	u8 checksum;
} __packed;

/*
 * the touch sensitivity command is not common to all displays.
 * because of the way it's integrated into the panel driver, it will stay here
 *	for the time being.  perhaps some time in the future it will be easier
 *	to break this out.
 */
struct hralvds_touch_sensitivity_cmd_struct {
	u8 cmd_id;
	union {
		u8 data[HRALVDS_DIAG_TOUCH_SENS_CMD_SIZE];
		struct {
			u8 level;
		} touch_sensitivity;
	};
	u8 checksum;
} __packed;

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;
	struct workqueue_struct *workq;
	struct delayed_work hralvds_timer_work;
	struct delayed_work hralvds_hdcp_work;
	struct delayed_work hralvds_hdcp_auth_retry_work;
	struct delayed_work hralvds_power_state_work;
	struct i2c_client *ser_i2c_client;
	struct i2c_client *deser_i2c_client;
	struct i2c_client *ts_i2c_client;
	struct input_dev *input_dev;
	unsigned int irq;
	unsigned int bad_irq_count;
	unsigned int irq_count;
	int dith;
	int supervisor_ok;
	int display_state;
	int display_detected;
	int dispc_gpio;
	int serializer_reset_gpio;
	volatile int power_state;
	int gradual_change_retries;
	int reset_retries;
	volatile int power_cmd_processed;
	atomic_t fast_init;
	unsigned poll_counter;
	unsigned supervisor_failure_counter;
	unsigned setup_tries;
	unsigned int hdcp_enable;
	unsigned int hdcp_previous_state;

	unsigned long timer_worker_delay;	/* delay for timer_worker */
	u32 deser_phys;		/* physical (real) address of deser */
	u32 deser_alias;	/* alias used on this side of FPDlink */

#if defined HRALVDS_LOG_DEBUG
	unsigned long dbg_lvl;	/* debug level */
#endif

	/* last value of the Status Command (HRALVDS_REPORT_ID_STATUS) */
	u8 event_status[HRALVDS_REPORT_ID_STATUS_SIZE];
	struct mutex event_status_lock;

	/* Shadowed power command register */
	struct hralvds_power_cmd_struct cmd_power;
	struct mutex cmd_power_lock;

	/* Shadowed video command register */
	struct hralvds_video_cmd_struct cmd_video;
	struct mutex cmd_video_lock;

	/* Shadowed radio frequency command register */
	struct hralvds_radio_freq_cmd_struct cmd_radio_freq;
	struct mutex cmd_radio_freq_lock;

	/* Shadowed touch sensitivity command register */
	struct hralvds_touch_sensitivity_cmd_struct cmd_touch_sensitivity;
	struct mutex cmd_touch_sensitivity_lock;

	/*
	 * Power command to send to disable all illumination prior to
	 * shutting the panel down.
	 */
	struct hralvds_power_cmd_struct cmd_power_off;

	void *hralvds_panel_data;

	bool pixel_clock_rising_edge;
};


static inline int i2c_write_le8(struct i2c_client *client, unsigned addr,
				unsigned data)
{
	int ret = i2c_smbus_write_byte_data(client, addr, data);
	if (ret)
		dev_dbg(&client->dev, "Failed to write 0x%02x to 0x%02x",
			data, addr);
	return ret;
}

static inline int i2c_read_le8(struct i2c_client *client, unsigned addr)
{
	int ret = i2c_smbus_read_byte_data(client, addr);
	if (ret < 0)
		dev_dbg(&client->dev, "Failed to read 0x%02x, Error = %d",
			addr, ret);
	return ret;
}

/*
 * functions exposed by the panel display drivers (THRA, TJBA, etc.) that need
 * 	to be called from panel-hralvds.c
 */
extern int hralvds_panel_probe(struct panel_drv_data *ddata);
extern int hralvds_panel_irq(struct panel_drv_data *ddata);

/*
 * alpin720_i2c_cmdrsp:
 *
 * send a command to the i2c bus and receive a response.
 *		wbuf	pointer to buffer to transmit from
 *		wlen	number of bytes from wbuf to send
 *		rbuf	pointer to buffer to receive into
 *		rlen	number of bytes to receive into the buffer
 *
 *	i2c_transfer returns # of messages sent on success,
 *		or negative errno on failure.
 */
static inline int hralvds_i2c_cmdrsp(struct i2c_client *client,
			__u8 *wbuf, __u16 wlen, __u8 *rbuf, __u16 rlen)
{
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = wbuf;
	msg[0].len = wlen;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = rbuf;
	msg[1].len = rlen;

	return i2c_transfer(client->adapter, msg,
		sizeof(msg) / sizeof(msg[0]));
}

/*
 * alpin720_i2c_cmd:
 *
 * write a buffer to the i2c client.
 *		client	i2c client to send to
 *		wbuf		pointer to buffer to transmit from
 *		wlen		number of bytes from wbuf to send
 *
 *	i2c_transfer returns # of messages sent on success,
 *		or negative errno on failure.
 */
static inline int hralvds_i2c_cmd(struct i2c_client *client,
				__u8 *wbuf, __u16 wlen)
{
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = wbuf;
	msg.len = wlen;

	return i2c_transfer(client->adapter, &msg, 1);
}

/*
 * hralvds_chksum:
 *
 * compute the hralvds checksum over everything sent in the wbuf (everything
 *	sent to the hralvds) and everything received in the rbuf (including
 *	the checksum that hralvds sent).  a valid message sequence will
 *	produce a checksum of zero.
 * example:
 *	host protocol traffic		display protocol traffic
 *	---------------------		----------------------------
 *	CMD D0			->
 *				<-	D1, D2, ... DN, CKS
 *
 * in this example, the sum of
 *		CMD + D0 + D1 + D2 + ... + DN + CKS
 * will be zero if the message was transmitted and received correctly.
 */
static inline __u8 hralvds_cksum(const __u8 *wbuf, size_t wbuflen,
				const __u8 *rbuf, size_t rbuflen)
{
	__u8 cksum = 0;
	size_t i;

	if (wbuf)
		for (i = 0; i < wbuflen; i++)
			cksum += wbuf[i];

	if (rbuf)
		for (i = 0; i < rbuflen; i++)
			cksum += rbuf[i];

	return cksum;
}

#if defined HRALVDS_LOG_DEBUG
/*
 * hralvds_dump_msg:
 *
 * dev is the device to use to print.
 * funcname is the name of the function to report, msg is the bytes to dump,
 *		msglen is the # of bytes in msg to dump, and buf is the buffer
 *		buffer that will be needed to print.
 */
static inline void hralvds_dump_msg(struct device *dev, const char *funcname,
				const char *msg, size_t msglen,
				char *buf, size_t buflen)
{
	int len = 0;
	unsigned i;

	len = scnprintf(buf, buflen, "%s ", funcname);
	for (i = 0; i < msglen && len < buflen - 3; i++)
		len += scnprintf(buf + len, buflen - len, "%02hhx ", msg[i]);
	if (len < buflen - 2)
		len += scnprintf(buf + len, buflen - len, "\n");
	dev_info(dev, buf);
}
#endif

extern void hralvds_on_finish_gradual_change(struct panel_drv_data *ddata);

extern int hralvds_process_input_event(struct panel_drv_data *ddata);

#endif	/* __PANEL_HRALVDS_H */
