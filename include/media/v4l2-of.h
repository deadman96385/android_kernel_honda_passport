/*
 * V4L2 OF binding parsing library
 *
 * Copyright (C) 2012 - 2013 Samsung Electronics Co., Ltd.
 * Author: Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Copyright (C) 2012 Renesas Electronics Corp.
 * Author: Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 */
#ifndef _V4L2_OF_H
#define _V4L2_OF_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/of_graph.h>

#include <media/v4l2-mediabus.h>

struct device_node;

/**
 * struct v4l2_of_bus_mipi_csi2 - MIPI CSI-2 bus data structure
 * @flags: media bus (V4L2_MBUS_*) flags
 * @data_lanes: an array of physical data lane indexes
 * @clock_lane: physical lane index of the clock lane
 * @num_data_lanes: number of data lanes
 * @lane_polarities: polarity of the lanes. The order is the same of
 *		   the physical lanes.
 */
struct v4l2_of_bus_mipi_csi2 {
	unsigned int flags;
	unsigned char data_lanes[4];
	unsigned char clock_lane;
	unsigned short num_data_lanes;
	bool lane_polarities[5];
};

/**
 * struct v4l2_of_bus_parallel - parallel data bus data structure
 * @flags: media bus (V4L2_MBUS_*) flags
 * @bus_width: bus width in bits
 * @data_shift: data shift in bits
 */
struct v4l2_of_bus_parallel {
	unsigned int flags;
	unsigned char bus_width;
	unsigned char data_shift;
	unsigned char num_channels;
	unsigned char pixmux;
	unsigned char channels[16];
};

/**
 * struct v4l2_of_endpoint - the endpoint data structure
 * @base: struct of_endpoint containing port, id, and local of_node
 * @bus_type: bus type
 * @bus: bus configuration data structure
 * @head: list head for this structure
 */
struct v4l2_of_endpoint {
	struct of_endpoint base;
	enum v4l2_mbus_type bus_type;
	union {
		struct v4l2_of_bus_parallel parallel;
		struct v4l2_of_bus_mipi_csi2 mipi_csi2;
	} bus;
	struct list_head head;
};

#ifdef CONFIG_OF
int v4l2_of_parse_endpoint(const struct device_node *node,
			   struct v4l2_of_endpoint *endpoint);
#else /* CONFIG_OF */

static inline int v4l2_of_parse_endpoint(const struct device_node *node,
					struct v4l2_of_endpoint *link)
{
	return -ENOSYS;
}

#endif /* CONFIG_OF */

#endif /* _V4L2_OF_H */
