/* SPDX-License-Identifier: GPL-2.0 */
/*
 * u_uac2.h
 *
 * Utility definitions for UAC2 function
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Andrzej Pietrasiewicz <andrzejtp2010@gmail.com>
 */

#ifndef U_UAC2_H
#define U_UAC2_H

#include <linux/usb/composite.h>
#include "uac_common.h"

#define UAC2_DEF_PCHMASK 0x3
#define UAC2_DEF_PHSBINT 0
#define UAC2_DEF_CCHMASK 0x3
#define UAC2_DEF_CHSBINT 0
#define UAC2_DEF_CSYNC		USB_ENDPOINT_SYNC_ASYNC

#define UAC2_DEF_SRATE 48000
#define UAC2_DEF_SSIZE 2

#define UAC2_DEF_REQ_NUM 2
#define UAC2_DEF_INT_REQ_NUM	10

struct f_uac2_opts {
	struct usb_function_instance	func_inst;
	int				p_chmask;
	u8				p_hs_bint;
	int				c_chmask;
	int				c_sync;
	u8				c_hs_bint;

	int				srates[UAC_MAX_RATES];
	int				ssize;

	int				req_number;
	int				fb_max;
	bool			bound;

	char			function_name[32];

	struct mutex			lock;
	int				refcnt;
};

#endif
