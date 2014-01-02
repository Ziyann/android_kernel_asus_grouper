/*
 * ltr659ps.h -- LTR-659PS-01 proximity sensor header file.
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#ifndef _LTR659PS_H
#define _LTR659PS_H

struct ltr659_platform_data {
	int gpio;
	int default_ps_lowthreshold;
	int default_ps_highthreshold;
};

#endif  /* #ifndef _LTR659PS_H */
