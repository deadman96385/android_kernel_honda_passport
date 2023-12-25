/*********************************************************
*
*  Copyright (c) 2014-2016 Honda R&D Americas, Inc.
*  All rights reserved. You may not copy, distribute, publicly display,
*  create derivative works from or otherwise use or modify this
*  software without first obtaining a license from Honda R&D Americas, Inc.
*
*********************************************************/

#ifndef __HRALVDS_PROTO_H
#define __HRALVDS_PROTO_H

#include "deserializer.h"

/* Status register defines */
/* DES928_MB0 is a scratch "mailbox" register in the deserializer */
#define HRALVDS_INIT_STATUS_REG		DES928_MB0
#define HRALVDS_INIT_STATUS_REG_MAX_POLL	5
#define HRALVDS_INIT_STATUS_REG_INIT_COMPLETE_BIT	(1 << 0)
#define HRALVDS_INIT_STATUS_REG_EST_COMPLETE_BIT	(1 << 1)

#define HRALVDS_SUPERVISOR_CMD 	0x3C
#define HRALVDS_SUPERVISOR_CMD_SIZE 	1

#define HRALVDS_SUPERVISOR_RSP 	0xA5
#define HRALVDS_SUPERVISOR_RSP_SIZE 	1

#define HRALVDS_SUPERVISOR_FAILURE_LIMIT	5

#define HRALVDS_VERSION_CMD	0x30
#define HRALVDS_VERSION_RSP_SIZE	16

#define HRALVDS_DIAG_STATUS_CMD	0x2c
#define HRALVDS_DIAG_STATUS_RSP_SIZE	64

#define HRALVDS_DIAG_TOUCH_CAL_CMD	0x0c
#define HRALVDS_DIAG_TOUCH_CAL_CMD_SIZE	40

#define HRALVDS_DIAG_TOUCH_SENS_CMD	0x0d
#define HRALVDS_DIAG_TOUCH_SENS_CMD_SIZE	1

#define HRALVDS_POWER_CMD	0x14
#define HRALVDS_POWER_CMD_SIZE	8

#define HRALVDS_VIDEO_CMD	0x18
#define HRALVDS_VIDEO_CMD_SIZE	4

#define HRALVDS_RADIO_FREQ_CMD	0x15
#define HRALVDS_RADIO_FREQ_CMD_SIZE	4

#define HRALVDS_REPORT_ID_CMD	0x9C

#define HRALVDS_REPORT_ID_KEY	0x80
#define HRALVDS_REPORT_ID_KEY_SIZE	8

#define HRALVDS_REPORT_ID_KNOB	0x83
#define HRALVDS_REPORT_ID_KNOB_SIZE	5

#define HRALVDS_REPORT_ID_TOUCH	0x84
#define HRALVDS_REPORT_ID_TOUCH_SIZE	25

#define HRALVDS_REPORT_ID_STATUS	0x88
#define HRALVDS_REPORT_ID_STATUS_SIZE	2

#define HRALVDS_NUM_MT_SLOTS	5

/*
 * in the KEY INTERRUPT response from the touch screen, these are the offsets
 *		of the individual key statuses.
 */
#define HRALVDS_DAY_NIGHT_KEY_OFFS	0
#define HRALVDS_POWER_KNOB_KEY_OFFS	1
#define HRALVDS_HOME_KEY_OFFS	2
#define HRALVDS_MENU_KEY_OFFS	3
#define HRALVDS_BACK_KEY_OFFS	4
#define HRALVDS_KEY_MAX_OFFS	5

#define HRALVDS_KEY_NOT_PRESSED	0
#define HRALVDS_KEY_PRESSED	1

#endif /* __HRALVDS_PROTO_H */
