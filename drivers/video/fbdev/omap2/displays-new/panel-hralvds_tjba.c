/*
 * Copyright (C) 2013 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Copyright (C) 2014 Denso International America */

/*********************************************************
*
*  This file modified by Honda R&D Americas, Inc. on November 24, 2014
*
*  All modifications made by Honda R&D Americas, Inc.
*  are Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*
*  Honda R&D Americas, Inc. hereby licenses those modifications
*  under the terms set forth in the file HONDA-NOTICE
*  located in the root of the directory /vendor/honda
*
*********************************************************/

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/time.h>

#include <video/omapdss.h>

#include "serializer.h"
#include "deserializer.h"
#include "panel-hralvds.h"
#include "panel-hralvds_tjba.h"
#include "panel-hralvds_thra.h"
#include "hralvds-proto.h"

#if 0
struct omap_video_timings hralvds_video_timings = {
	.pixelclock	= 74250000,			/* Hz */

	.x_res		= HRALVDS_THRA_WIDTH,
	.hfp		= 90,
	.hbp		= 90,
	.hsw		= 40,

	.y_res		= HRALVDS_THRA_HEIGHT,
	.vfp		= 49,
	.vbp		= 51,
	.vsw		= 5,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
};

#else

/* panel-hralvds.c's hralvds_i2c_probe function will refer to this object. */
struct omap_video_timings hralvds_video_timings = {
	/*	This is the result of the calculation, in Hz:
		.pixelclock	= 82446624,
	*/
	.pixelclock	= (HRALVDS_TJBA_WIDTH + 46 + 4 + 6) * (HRALVDS_TJBA_HEIGHT + 10 + 1 + 1) * 57,

	.x_res		= HRALVDS_TJBA_WIDTH,
	.hfp		= 46,
	.hbp		= 4,
	.hsw		= 6,

	.y_res		= HRALVDS_TJBA_HEIGHT,
	.vfp		= 10,
	.vbp		= 1,
	.vsw		= 1,

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
};

#endif

/* handle a status event from the hralvds display */
static void hralvds_status_event(struct panel_drv_data *ddata)
{
	/* buffer to send contains only the REPORT ID STATUS command byte. */
	__u8 wbuf[] = { HRALVDS_REPORT_ID_STATUS };
	/* buffer to receive the response in.  '+ 1' for the checksum. */
	__u8 rbuf[HRALVDS_REPORT_ID_STATUS_SIZE + 1] = { 0 };
	int r;

	/* read the status event and get the information from the hralvds. */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
				wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));

	if (r > 0) {
		__u8 cksum;

#if defined HRALVDS_LOG_DEBUG
		/*
		 * possibly dump the received buffer (done before
		 * checksum).
		 */
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
			char buf[(sizeof(__func__) + 1) +
					(sizeof(rbuf) * 3) + 2];
			hralvds_dump_msg(&ddata->ts_i2c_client->dev,
						__func__,
						rbuf, sizeof(rbuf),
						buf, sizeof(buf));
		}
#endif

		/* compute the checksum on the message.  0 == valid message. */
		cksum = hralvds_cksum(wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));

		if (0 == cksum) {
			/* Handle gradual time change complete */
			if (rbuf[0] & (1 << 6))
				hralvds_on_finish_gradual_change(ddata);

			mutex_lock(&ddata->event_status_lock);
			ddata->event_status[0] = rbuf[0];
			ddata->event_status[1] = rbuf[1];
			mutex_unlock(&ddata->event_status_lock);
			sysfs_notify(&ddata->ts_i2c_client->dev.kobj, NULL,
					"hralvds_event_status");
		}
	}
}

static void hralvds_dispatch_event(struct panel_drv_data *ddata,
	__u8 *rbuf, size_t rbuflen)
{
#if defined HRALVDS_LOG_DEBUG
	/*
	 * possibly dump the received buffer (done before checksum).
	 * this can delay processing and communication with the display,
	 *	so use it carefully.
	 */
	if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_DUMP) {
		char buf[(sizeof(__func__) + 1) + (rbuflen * 3) + 2];

		hralvds_dump_msg(&ddata->ts_i2c_client->dev,
					__func__,
					rbuf, rbuflen ,
					buf, sizeof(buf));
	}
#endif

	/*
	 * valid checksum, see what the
	 *	touchscreen wants to tell us.
	 */
	switch (rbuf[0]) {
	case HRALVDS_REPORT_ID_STATUS:
		/* process the event before doing any debug */
		hralvds_status_event(ddata);

#if defined HRALVDS_LOG_DEBUG
		if (ddata->dbg_lvl & HRALVDS_DBG_LEVEL_EVENT)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s active: STATUS\n", __func__);

		if ((HRALVDS_REPORT_ID_STATUS_SIZE != rbuf[1]) &&
								ddata->dbg_lvl)
			dev_info(&ddata->ts_i2c_client->dev,
				"%s invalid length reported in "
				"STATUS event: %d, but should be %d\n",
				__func__, rbuf[1],
				HRALVDS_REPORT_ID_STATUS_SIZE);
#endif
		break;

	default:
		break;
	}
}

/*
 * hralvds_config_input_dev:
 *
 * configure the input device so that key, touch screen, knob and status
 *	events can go to the input system.
 *
 * THIS FUNCTION NEEDS TO BE MOVED TO THE PROPER DISPLAY-SPECIFIC DRIVER FILE.
 */
static int hralvds_config_input_dev(struct device *dev)
{
	int r;
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	ddata->input_dev = devm_input_allocate_device(dev);
	if (!ddata->input_dev)
		return -ENOMEM;

	ddata->input_dev->name = "HRALVDS THRA Touchscreen Panel";
	ddata->input_dev->id.bustype = BUS_I2C;
	ddata->input_dev->dev.parent = dev;
	input_set_drvdata(ddata->input_dev, ddata);

	input_set_abs_params(ddata->input_dev, ABS_X, 0,
					HRALVDS_TJBA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_Y, 0,
					HRALVDS_TJBA_HEIGHT, 0, 0);

	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_X, 0,
					HRALVDS_TJBA_WIDTH, 0, 0);
	input_set_abs_params(ddata->input_dev, ABS_MT_POSITION_Y, 0,
					HRALVDS_TJBA_HEIGHT, 0, 0);

	r = input_register_device(ddata->input_dev);

	return 1;
}

/*
 * called by hralvds_i2c_probe().
 */
int hralvds_panel_probe(struct panel_drv_data *ddata)
{
	int r;

	ddata->hralvds_panel_data =
		devm_kzalloc(&ddata->ts_i2c_client->dev,
			sizeof(struct hralvds_tjba_data), GFP_KERNEL);

	if (NULL == ddata->hralvds_panel_data)
		return -ENOMEM;

	r = hralvds_config_input_dev(&ddata->ts_i2c_client->dev);
	if (r >= 0)
	 	return 0;

	dev_info(&ddata->ts_i2c_client->dev,
			"Failed to allocate/configure input device.");
	kfree(ddata->hralvds_panel_data);
	return r;
}

int hralvds_panel_irq(struct panel_drv_data *ddata)
{
	int r;
	__u8 cksum;
	__u8 wbuf[1] = { HRALVDS_REPORT_ID_CMD };
	__u8 rbuf[2 + 1] = { 0 };

	/*
	 * read the report ID and get the information
	 *	from the hralvds.
	 */
	r = hralvds_i2c_cmdrsp(ddata->ts_i2c_client,
		wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));

	if (r > 0) {
		/*
		 * compute the checksum on the message.
		 *	0 == valid message.
		 */
		cksum = hralvds_cksum(
			wbuf, sizeof(wbuf),
			rbuf, sizeof(rbuf));

		if (0 == cksum)
			/*
			 * decode the event and dispatch it up
			 * 	to the event system.
			 */
			hralvds_dispatch_event(ddata, rbuf,
				sizeof(rbuf));
	}

	return 0;
}
