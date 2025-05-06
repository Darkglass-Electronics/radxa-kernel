// SPDX-License-Identifier: GPL-2.0+
/*
 * f_uac2.c -- USB Audio Class 2.0 Function
 *
 * Copyright (C) 2011
 *    Yadwinder Singh (yadi.brar01@gmail.com)
 *    Jaswinder Singh (jaswinder.singh@linaro.org)
 *
 * Copyright (C) 2020
 *    Ruslan Bilovol (ruslan.bilovol@gmail.com)
 */

#include <linux/usb/audio.h>
#include <linux/usb/audio-v2.h>
#include <linux/module.h>

#include "u_audio.h"

#include "u_uac2.h"

/* UAC2 spec: 4.1 Audio Channel Cluster Descriptor */
#define UAC2_CHANNEL_MASK 0x07FFFFFF

/*
 * The driver implements a simple UAC_2 topology.
 * USB-OUT -> IT_1 -> FU -> OT_3 -> ALSA_Capture
 * ALSA_Playback -> IT_2 -> FU -> OT_4 -> USB-IN
 */
#define USB_CLK_ID	(clk_src_desc.bClockID)

#define CONTROL_ABSENT	0
#define CONTROL_RDONLY	1
#define CONTROL_RDWR	3

#define CLK_FREQ_CTRL	0
#define CLK_VLD_CTRL	2

#define COPY_CTRL	0
#define CONN_CTRL	2
#define OVRLD_CTRL	4
#define CLSTR_CTRL	6
#define UNFLW_CTRL	8
#define OVFLW_CTRL	10

#define EPIN_EN(_opts) ((_opts)->p_chmask != 0)
#define EPOUT_EN(_opts) ((_opts)->c_chmask != 0)
#define EPOUT_FBACK_IN_EN(_opts) ((_opts)->c_sync == USB_ENDPOINT_SYNC_ASYNC)

struct f_uac2 {
	struct g_audio g_audio;
	u8 ac_intf, as_in_intf, as_out_intf;
	u8 ac_alt, as_in_alt, as_out_alt;	/* needed for get_alt() */

	struct usb_ctrlrequest setup_cr;	/* will be used in data stage */

	/* transient state, only valid during handling of a single control request */
	int clock_id;
};

static inline struct f_uac2 *func_to_uac2(struct usb_function *f)
{
	return container_of(f, struct f_uac2, g_audio.func);
}

static inline
struct f_uac2_opts *g_audio_to_uac2_opts(struct g_audio *agdev)
{
	return container_of(agdev->func.fi, struct f_uac2_opts, func_inst);
}

/* --------- USB Function Interface ------------- */

enum {
	STR_ASSOC,
	STR_TERMINAL1,
	STR_TERMINAL2,
};

static struct usb_string strings_fn[4] = {
	/* [STR_ASSOC].s = DYNAMIC, */
	/* [STR_TERMINAL1].s = DYNAMIC, */
	/* [STR_TERMINAL2].s = DYNAMIC, */
	{ },
};

static const char *const speed_names[] = {
	[USB_SPEED_UNKNOWN] = "UNKNOWN",
	[USB_SPEED_LOW] = "LS",
	[USB_SPEED_FULL] = "FS",
	[USB_SPEED_HIGH] = "HS",
	[USB_SPEED_WIRELESS] = "W",
	[USB_SPEED_SUPER] = "SS",
	[USB_SPEED_SUPER_PLUS] = "SS+",
};

static struct usb_gadget_strings str_fn = {
	.language = 0x0409,	/* en-us */
	.strings = strings_fn,
};

static struct usb_gadget_strings *fn_strings[] = {
	&str_fn,
	NULL,
};

static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface = 0,
	.bInterfaceCount = 3,
	.bFunctionClass = USB_CLASS_AUDIO,
	.bFunctionSubClass = UAC2_FUNCTION_SUBCLASS_UNDEFINED,
	.bFunctionProtocol = UAC_VERSION_2,
};

/* Audio Control Interface */
static struct usb_interface_descriptor std_ac_if_desc = {
	.bLength = sizeof std_ac_if_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	/* .bNumEndpoints = DYNAMIC */
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOCONTROL,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Clock source for IN traffic */
static struct uac_clock_source_descriptor clk_src_desc = {
	.bLength = sizeof clk_src_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC2_CLOCK_SOURCE,
	/* .bClockID = DYNAMIC */
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_FIXED,
	.bmControls = (CONTROL_RDWR << CLK_FREQ_CTRL),
	.bAssocTerminal = 0,
};

/* Input Terminal for USB_OUT */
static struct uac2_input_terminal_descriptor usb_out_it_desc = {
	.bLength = sizeof usb_out_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	/* .bTerminalID = DYNAMIC */
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	/* .bCSourceID = DYNAMIC */
	.iChannelNames = 0,
	.bmControls = cpu_to_le16(CONTROL_RDWR << COPY_CTRL),
};

/* Input Terminal for I/O-In */
static struct uac2_input_terminal_descriptor io_in_it_desc = {
	.bLength = sizeof io_in_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	/* .bTerminalID = DYNAMIC */
	.wTerminalType = cpu_to_le16(UAC_INPUT_TERMINAL_MICROPHONE),
	.bAssocTerminal = 0,
	/* .bCSourceID = DYNAMIC */
	.iChannelNames = 0,
	.bmControls = cpu_to_le16(CONTROL_RDWR << COPY_CTRL),
};

/* Ouput Terminal for USB_IN */
static struct uac2_output_terminal_descriptor usb_in_ot_desc = {
	.bLength = sizeof usb_in_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	/* .bTerminalID = DYNAMIC */
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	/* .bSourceID = DYNAMIC */
	/* .bCSourceID = DYNAMIC */
	.bmControls = cpu_to_le16(CONTROL_RDWR << COPY_CTRL),
};

/* Ouput Terminal for I/O-Out */
static struct uac2_output_terminal_descriptor io_out_ot_desc = {
	.bLength = sizeof io_out_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	/* .bTerminalID = DYNAMIC */
	.wTerminalType = cpu_to_le16(UAC_OUTPUT_TERMINAL_SPEAKER),
	.bAssocTerminal = 0,
	/* .bSourceID = DYNAMIC */
	/* .bCSourceID = DYNAMIC */
	.bmControls = cpu_to_le16(CONTROL_RDWR << COPY_CTRL),
};

static struct uac2_ac_header_descriptor ac_hdr_desc = {
	.bLength = sizeof ac_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_MS_HEADER,
	.bcdADC = cpu_to_le16(0x200),
	.bCategory = UAC2_FUNCTION_IO_BOX,
	/* .wTotalLength = DYNAMIC */
	.bmControls = 0,
};

/* Audio Streaming OUT Interface - Alt0 */
static struct usb_interface_descriptor std_as_out_if0_desc = {
	.bLength = sizeof std_as_out_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Streaming OUT Interface - Alt1 */
static struct usb_interface_descriptor std_as_out_if1_desc = {
	.bLength = sizeof std_as_out_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Stream OUT Intface Desc */
static struct uac2_as_header_descriptor as_out_hdr_desc = {
	.bLength = sizeof as_out_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_AS_GENERAL,
	/* .bTerminalLink = DYNAMIC */
	.bmControls = 0,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bmFormats = cpu_to_le32(UAC_FORMAT_TYPE_I_PCM),
	.iChannelNames = 0,
};

/* Audio USB_OUT Format */
static struct uac2_format_type_i_descriptor as_out_fmt1_desc = {
	.bLength = sizeof as_out_fmt1_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_FORMAT_TYPE,
	.bFormatType = UAC_FORMAT_TYPE_I,
};

/* STD AS ISO OUT Endpoint */
static struct usb_endpoint_descriptor fs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,
	/* .bmAttributes = DYNAMIC */
	/* .wMaxPacketSize = DYNAMIC */
	.bInterval = 1,
};

static struct usb_endpoint_descriptor hs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	/* .bmAttributes = DYNAMIC */
	/* .wMaxPacketSize = DYNAMIC */
	/* .bInterval = DYNAMIC */
};

static struct usb_endpoint_descriptor ss_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,
	/* .bmAttributes = DYNAMIC */
	/* .wMaxPacketSize = DYNAMIC */
	/* .bInterval = DYNAMIC */
};

static struct usb_ss_ep_comp_descriptor ss_epout_desc_comp = {
	.bLength		= sizeof(ss_epout_desc_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	/* wBytesPerInterval = DYNAMIC */
};

/* CS AS ISO OUT Endpoint */
static struct uac2_iso_endpoint_descriptor as_iso_out_desc = {
	.bLength = sizeof as_iso_out_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0,
};

/* STD AS ISO IN Feedback Endpoint */
static struct usb_endpoint_descriptor fs_epin_fback_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_USAGE_FEEDBACK,
	.wMaxPacketSize = cpu_to_le16(4),
	.bInterval = 1,
};

static struct usb_endpoint_descriptor hs_epin_fback_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_USAGE_FEEDBACK,
	.wMaxPacketSize = cpu_to_le16(4),
	.bInterval = 4,
};

static struct usb_endpoint_descriptor ss_epin_fback_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_USAGE_FEEDBACK,
	.wMaxPacketSize = cpu_to_le16(4),
	.bInterval = 4,
};

static struct usb_ss_ep_comp_descriptor ss_epin_fback_desc_comp = {
	.bLength		= sizeof(ss_epin_fback_desc_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	.wBytesPerInterval	= cpu_to_le16(4),
};


/* Audio Streaming IN Interface - Alt0 */
static struct usb_interface_descriptor std_as_in_if0_desc = {
	.bLength = sizeof std_as_in_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Streaming IN Interface - Alt1 */
static struct usb_interface_descriptor std_as_in_if1_desc = {
	.bLength = sizeof std_as_in_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Stream IN Intface Desc */
static struct uac2_as_header_descriptor as_in_hdr_desc = {
	.bLength = sizeof as_in_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_AS_GENERAL,
	/* .bTerminalLink = DYNAMIC */
	.bmControls = 0,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bmFormats = cpu_to_le32(UAC_FORMAT_TYPE_I_PCM),
	.iChannelNames = 0,
};

/* Audio USB_IN Format */
static struct uac2_format_type_i_descriptor as_in_fmt1_desc = {
	.bLength = sizeof as_in_fmt1_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_FORMAT_TYPE,
	.bFormatType = UAC_FORMAT_TYPE_I,
};

/* STD AS ISO IN Endpoint */
static struct usb_endpoint_descriptor fs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	/* .wMaxPacketSize = DYNAMIC */
	.bInterval = 1,
};

static struct usb_endpoint_descriptor hs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	/* .wMaxPacketSize = DYNAMIC */
	/* .bInterval = DYNAMIC */
};

static struct usb_endpoint_descriptor ss_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	/* .wMaxPacketSize = DYNAMIC */
	/* .bInterval = DYNAMIC */
};

static struct usb_ss_ep_comp_descriptor ss_epin_desc_comp = {
	.bLength		= sizeof(ss_epin_desc_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	/* wBytesPerInterval = DYNAMIC */
};

/* CS AS ISO IN Endpoint */
static struct uac2_iso_endpoint_descriptor as_iso_in_desc = {
	.bLength = sizeof as_iso_in_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0,
};

static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	(struct usb_descriptor_header *)&fs_epin_fback_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	(struct usb_descriptor_header *)&hs_epin_fback_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

static struct usb_descriptor_header *ss_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&ss_epout_desc,
	(struct usb_descriptor_header *)&ss_epout_desc_comp,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	(struct usb_descriptor_header *)&ss_epin_fback_desc,
	(struct usb_descriptor_header *)&ss_epin_fback_desc_comp,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&ss_epin_desc,
	(struct usb_descriptor_header *)&ss_epin_desc_comp,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

struct cntrl_cur_lay2 {
	__le16	wCUR;
};

struct cntrl_range_lay2 {
	__le16	wNumSubRanges;
	__le16	wMIN;
	__le16	wMAX;
	__le16	wRES;
} __packed;

struct cntrl_cur_lay3 {
	__le32	dCUR;
};

struct cntrl_subrange_lay3 {
	__le32	dMIN;
	__le32	dMAX;
	__le32	dRES;
} __packed;

#define ranges_lay3_size(c) (sizeof(c.wNumSubRanges)	\
		+ le16_to_cpu(c.wNumSubRanges)		\
		* sizeof(struct cntrl_subrange_lay3))

#define DECLARE_UAC2_CNTRL_RANGES_LAY3(k, n)		\
	struct cntrl_ranges_lay3_##k {			\
	__le16	wNumSubRanges;				\
	struct cntrl_subrange_lay3 r[n];		\
} __packed

DECLARE_UAC2_CNTRL_RANGES_LAY3(srates, UAC_MAX_RATES);

static int get_max_srate(const int *srates)
{
	int i, max_srate = 0;

	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (srates[i] == 0)
			break;
		if (srates[i] > max_srate)
			max_srate = srates[i];
	}
	return max_srate;
}

static int get_max_bw_for_bint(const struct f_uac2_opts *uac2_opts,
	u8 bint, unsigned int factor, bool is_playback)
{
	int chmask, srate, ssize;
	u16 max_size_bw;

	srate = get_max_srate(uac2_opts->srates);
	ssize = uac2_opts->ssize;

	if (is_playback) {
		chmask = uac2_opts->p_chmask;
	} else {
		chmask = uac2_opts->c_chmask;
	}

	if (is_playback || (uac2_opts->c_sync == USB_ENDPOINT_SYNC_ASYNC)) {
		// playback is always async, capture only when configured
		// Win10 requires max packet size + 1 frame
		srate = srate * (1000 + uac2_opts->fb_max) / 1000;
		// updated srate is always bigger, therefore DIV_ROUND_UP always yields +1
		max_size_bw = num_channels(chmask) * ssize *
			(DIV_ROUND_UP(srate, factor / (1 << (bint - 1))));
	} else {
		// adding 1 frame provision for Win10
		max_size_bw = num_channels(chmask) * ssize *
			(DIV_ROUND_UP(srate, factor / (1 << (bint - 1))) + 1);
	}
	return max_size_bw;
}

static int set_ep_max_packet_size_bint(struct device *dev, const struct f_uac2_opts *uac2_opts,
	struct usb_endpoint_descriptor *ep_desc,
	enum usb_device_speed speed, bool is_playback)
{
	u16 max_size_bw, max_size_ep;
	u8 bint, opts_bint;
	char *dir;

	switch (speed) {
	case USB_SPEED_FULL:
		max_size_ep = 1023;
		// fixed
		bint = ep_desc->bInterval;
		max_size_bw = get_max_bw_for_bint(uac2_opts, bint, 1000, is_playback);
		break;

	case USB_SPEED_HIGH:
	case USB_SPEED_SUPER:
		max_size_ep = 1024;
		if (is_playback)
			opts_bint = uac2_opts->p_hs_bint;
		else
			opts_bint = uac2_opts->c_hs_bint;

		if (opts_bint > 0) {
			/* fixed bint */
			bint = opts_bint;
			max_size_bw = get_max_bw_for_bint(uac2_opts, bint, 8000, is_playback);
		} else {
			/* checking bInterval from 4 to 1 whether the required bandwidth fits */
			for (bint = 4; bint > 0; --bint) {
				max_size_bw = get_max_bw_for_bint(
					uac2_opts, bint, 8000, is_playback);
				if (max_size_bw <= max_size_ep)
					break;
			}
		}
		break;

	default:
		return -EINVAL;
	}

	if (is_playback)
		dir = "Playback";
	else
		dir = "Capture";

	if (max_size_bw <= max_size_ep)
		dev_dbg(dev,
			"%s %s: Would use wMaxPacketSize %d and bInterval %d\n",
			speed_names[speed], dir, max_size_bw, bint);
	else {
		dev_warn(dev,
			"%s %s: Req. wMaxPacketSize %d at bInterval %d > max ISOC %d, may drop data!\n",
			speed_names[speed], dir, max_size_bw, bint, max_size_ep);
		max_size_bw = max_size_ep;
	}

	ep_desc->wMaxPacketSize = cpu_to_le16(max_size_bw);
	ep_desc->bInterval = bint;

	return 0;
}

/* Use macro to overcome line length limitation */
#define USBDHDR(p) (struct usb_descriptor_header *)(p)

static void setup_headers(struct f_uac2_opts *opts,
			  struct usb_descriptor_header **headers,
			  enum usb_device_speed speed)
{
	struct usb_ss_ep_comp_descriptor *epout_desc_comp = NULL;
	struct usb_ss_ep_comp_descriptor *epin_desc_comp = NULL;
	struct usb_ss_ep_comp_descriptor *epin_fback_desc_comp = NULL;
	struct usb_endpoint_descriptor *epout_desc;
	struct usb_endpoint_descriptor *epin_desc;
	struct usb_endpoint_descriptor *epin_fback_desc;
	int i;

	switch (speed) {
	case USB_SPEED_FULL:
		epout_desc = &fs_epout_desc;
		epin_desc = &fs_epin_desc;
		epin_fback_desc = &fs_epin_fback_desc;
		break;
	case USB_SPEED_HIGH:
		epout_desc = &hs_epout_desc;
		epin_desc = &hs_epin_desc;
		epin_fback_desc = &hs_epin_fback_desc;
		break;
	default:
		epout_desc = &ss_epout_desc;
		epin_desc = &ss_epin_desc;
		epout_desc_comp = &ss_epout_desc_comp;
		epin_desc_comp = &ss_epin_desc_comp;
		epin_fback_desc = &ss_epin_fback_desc;
		epin_fback_desc_comp = &ss_epin_fback_desc_comp;
	}

	i = 0;
	headers[i++] = USBDHDR(&iad_desc);
	headers[i++] = USBDHDR(&std_ac_if_desc);
	headers[i++] = USBDHDR(&ac_hdr_desc);
	if (EPIN_EN(opts) || EPOUT_EN(opts))
		headers[i++] = USBDHDR(&clk_src_desc);
	if (EPOUT_EN(opts))
		headers[i++] = USBDHDR(&usb_out_it_desc);

	if (EPIN_EN(opts)) {
		headers[i++] = USBDHDR(&io_in_it_desc);
		headers[i++] = USBDHDR(&usb_in_ot_desc);
	}

	if (EPOUT_EN(opts))
		headers[i++] = USBDHDR(&io_out_ot_desc);

	if (EPOUT_EN(opts)) {
		headers[i++] = USBDHDR(&std_as_out_if0_desc);
		headers[i++] = USBDHDR(&std_as_out_if1_desc);
		headers[i++] = USBDHDR(&as_out_hdr_desc);
		headers[i++] = USBDHDR(&as_out_fmt1_desc);
		headers[i++] = USBDHDR(epout_desc);
		if (epout_desc_comp)
			headers[i++] = USBDHDR(epout_desc_comp);

		headers[i++] = USBDHDR(&as_iso_out_desc);

		if (EPOUT_FBACK_IN_EN(opts)) {
			headers[i++] = USBDHDR(epin_fback_desc);
			if (epin_fback_desc_comp)
				headers[i++] = USBDHDR(epin_fback_desc_comp);
		}
	}

	if (EPIN_EN(opts)) {
		headers[i++] = USBDHDR(&std_as_in_if0_desc);
		headers[i++] = USBDHDR(&std_as_in_if1_desc);
		headers[i++] = USBDHDR(&as_in_hdr_desc);
		headers[i++] = USBDHDR(&as_in_fmt1_desc);
		headers[i++] = USBDHDR(epin_desc);
		if (epin_desc_comp)
			headers[i++] = USBDHDR(epin_desc_comp);

		headers[i++] = USBDHDR(&as_iso_in_desc);
	}
	headers[i] = NULL;
}

static void setup_descriptor(struct f_uac2_opts *opts)
{
	/* patch descriptors */
	int i = 1; /* ID's start with 1 */

	if (EPOUT_EN(opts))
		usb_out_it_desc.bTerminalID = i++;
	if (EPIN_EN(opts))
		io_in_it_desc.bTerminalID = i++;
	if (EPOUT_EN(opts))
		io_out_ot_desc.bTerminalID = i++;
	if (EPIN_EN(opts))
		usb_in_ot_desc.bTerminalID = i++;
	if (EPIN_EN(opts))
		clk_src_desc.bClockID = i++;

	usb_out_it_desc.bCSourceID = clk_src_desc.bClockID;

	usb_in_ot_desc.bSourceID = io_in_it_desc.bTerminalID;

	usb_in_ot_desc.bCSourceID = clk_src_desc.bClockID;
	io_in_it_desc.bCSourceID = clk_src_desc.bClockID;
	io_out_ot_desc.bCSourceID = clk_src_desc.bClockID;

	io_out_ot_desc.bSourceID = usb_out_it_desc.bTerminalID;

	as_out_hdr_desc.bTerminalLink = usb_out_it_desc.bTerminalID;
	as_in_hdr_desc.bTerminalLink = usb_in_ot_desc.bTerminalID;

	iad_desc.bInterfaceCount = 1;
	ac_hdr_desc.wTotalLength = cpu_to_le16(sizeof(ac_hdr_desc));

	if (EPOUT_EN(opts) || EPIN_EN(opts)) {
		u16 len = le16_to_cpu(ac_hdr_desc.wTotalLength);

		len += sizeof(clk_src_desc);
		ac_hdr_desc.wTotalLength = cpu_to_le16(len);
	}
	if (EPIN_EN(opts)) {
		u16 len = le16_to_cpu(ac_hdr_desc.wTotalLength);

		len += sizeof(usb_in_ot_desc);
		len += sizeof(io_in_it_desc);
		ac_hdr_desc.wTotalLength = cpu_to_le16(len);
		iad_desc.bInterfaceCount++;
	}
	if (EPOUT_EN(opts)) {
		u16 len = le16_to_cpu(ac_hdr_desc.wTotalLength);

		len += sizeof(usb_out_it_desc);
		len += sizeof(io_out_ot_desc);
		ac_hdr_desc.wTotalLength = cpu_to_le16(len);
		iad_desc.bInterfaceCount++;
	}

	setup_headers(opts, fs_audio_desc, USB_SPEED_FULL);
	setup_headers(opts, hs_audio_desc, USB_SPEED_HIGH);
	setup_headers(opts, ss_audio_desc, USB_SPEED_SUPER);
}

static int afunc_validate_opts(struct g_audio *agdev, struct device *dev)
{
	struct f_uac2_opts *opts = g_audio_to_uac2_opts(agdev);
	const char *msg = NULL;

	if (!opts->p_chmask && !opts->c_chmask)
		msg = "no playback and capture channels";
	else if (opts->p_chmask & ~UAC2_CHANNEL_MASK)
		msg = "unsupported playback channels mask";
	else if (opts->c_chmask & ~UAC2_CHANNEL_MASK)
		msg = "unsupported capture channels mask";
	else if ((opts->ssize < 1) || (opts->ssize > 4))
		msg = "incorrect sample size";
	else if (!opts->srates[0])
		msg = "incorrect sampling rate";

	else if ((opts->p_hs_bint < 0) || (opts->p_hs_bint > 4))
		msg = "incorrect playback HS/SS bInterval (1-4: fixed, 0: auto)";
	else if ((opts->c_hs_bint < 0) || (opts->c_hs_bint > 4))
		msg = "incorrect capture HS/SS bInterval (1-4: fixed, 0: auto)";

	if (msg) {
		dev_err(dev, "Error: %s\n", msg);
		return -EINVAL;
	}

	return 0;
}

static int
afunc_bind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct f_uac2 *uac2 = func_to_uac2(fn);
	struct g_audio *agdev = func_to_g_audio(fn);
	struct usb_composite_dev *cdev = cfg->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &gadget->dev;
	struct f_uac2_opts *uac2_opts = g_audio_to_uac2_opts(agdev);
	struct usb_string *us;
	int ret;

	ret = afunc_validate_opts(agdev, dev);
	if (ret)
		return ret;

	strings_fn[STR_ASSOC].s = uac2_opts->function_name;
	strings_fn[STR_TERMINAL1].s = uac2_opts->function_name;
	strings_fn[STR_TERMINAL2].s = uac2_opts->function_name;

	us = usb_gstrings_attach(cdev, fn_strings, ARRAY_SIZE(strings_fn));
	if (IS_ERR(us))
		return PTR_ERR(us);

	iad_desc.iFunction = us[STR_ASSOC].id;
	usb_out_it_desc.iTerminal = us[STR_TERMINAL1].id;
	io_in_it_desc.iTerminal = us[STR_TERMINAL1].id;
	usb_in_ot_desc.iTerminal = us[STR_TERMINAL2].id;
	io_out_ot_desc.iTerminal = us[STR_TERMINAL2].id;

	/* Initialize the configurable parameters */
	usb_out_it_desc.bNrChannels = num_channels(uac2_opts->c_chmask);
	usb_out_it_desc.bmChannelConfig = cpu_to_le32(uac2_opts->c_chmask);
	io_in_it_desc.bNrChannels = num_channels(uac2_opts->p_chmask);
	io_in_it_desc.bmChannelConfig = cpu_to_le32(uac2_opts->p_chmask);
	as_out_hdr_desc.bNrChannels = num_channels(uac2_opts->c_chmask);
	as_out_hdr_desc.bmChannelConfig = cpu_to_le32(uac2_opts->c_chmask);
	as_in_hdr_desc.bNrChannels = num_channels(uac2_opts->p_chmask);
	as_in_hdr_desc.bmChannelConfig = cpu_to_le32(uac2_opts->p_chmask);
	as_out_fmt1_desc.bSubslotSize = uac2_opts->ssize;
	as_out_fmt1_desc.bBitResolution = uac2_opts->ssize * 8;
	as_in_fmt1_desc.bSubslotSize = uac2_opts->ssize;
	as_in_fmt1_desc.bBitResolution = uac2_opts->ssize * 8;

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err_free_fu;
	}
	iad_desc.bFirstInterface = ret;

	std_ac_if_desc.bInterfaceNumber = ret;
	uac2->ac_intf = ret;
	uac2->ac_alt = 0;

	if (EPOUT_EN(uac2_opts)) {
		ret = usb_interface_id(cfg, fn);
		if (ret < 0) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			goto err_free_fu;
		}
		std_as_out_if0_desc.bInterfaceNumber = ret;
		std_as_out_if1_desc.bInterfaceNumber = ret;
		std_as_out_if1_desc.bNumEndpoints = 1;
		uac2->as_out_intf = ret;
		uac2->as_out_alt = 0;

		if (EPOUT_FBACK_IN_EN(uac2_opts)) {
			fs_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC;
			hs_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC;
			ss_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC;
			std_as_out_if1_desc.bNumEndpoints++;
		} else {
			fs_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE;
			hs_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE;
			ss_epout_desc.bmAttributes =
			  USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE;
		}
	}

	if (EPIN_EN(uac2_opts)) {
		ret = usb_interface_id(cfg, fn);
		if (ret < 0) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			goto err_free_fu;
		}
		std_as_in_if0_desc.bInterfaceNumber = ret;
		std_as_in_if1_desc.bInterfaceNumber = ret;
		uac2->as_in_intf = ret;
		uac2->as_in_alt = 0;
	}

	std_ac_if_desc.bNumEndpoints = 0;

	hs_epin_desc.bInterval = uac2_opts->p_hs_bint;
	ss_epin_desc.bInterval = uac2_opts->p_hs_bint;
	hs_epout_desc.bInterval = uac2_opts->c_hs_bint;
	ss_epout_desc.bInterval = uac2_opts->c_hs_bint;

	/* Calculate wMaxPacketSize according to audio bandwidth */
	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &fs_epin_desc,
					USB_SPEED_FULL, true);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &fs_epout_desc,
					USB_SPEED_FULL, false);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &hs_epin_desc,
					USB_SPEED_HIGH, true);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &hs_epout_desc,
					USB_SPEED_HIGH, false);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &ss_epin_desc,
					USB_SPEED_SUPER, true);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	ret = set_ep_max_packet_size_bint(dev, uac2_opts, &ss_epout_desc,
					USB_SPEED_SUPER, false);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}

	if (EPOUT_EN(uac2_opts)) {
		agdev->out_ep = usb_ep_autoconfig(gadget, &fs_epout_desc);
		if (!agdev->out_ep) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			ret = -ENODEV;
			goto err_free_fu;
		}
		if (EPOUT_FBACK_IN_EN(uac2_opts)) {
			agdev->in_ep_fback = usb_ep_autoconfig(gadget,
						       &fs_epin_fback_desc);
			if (!agdev->in_ep_fback) {
				dev_err(dev, "%s:%d Error!\n",
					__func__, __LINE__);
				ret = -ENODEV;
				goto err_free_fu;
			}
		}
	}

	if (EPIN_EN(uac2_opts)) {
		agdev->in_ep = usb_ep_autoconfig(gadget, &fs_epin_desc);
		if (!agdev->in_ep) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			ret = -ENODEV;
			goto err_free_fu;
		}
	}

	agdev->in_ep_maxpsize = max_t(u16,
				le16_to_cpu(fs_epin_desc.wMaxPacketSize),
				le16_to_cpu(hs_epin_desc.wMaxPacketSize));
	agdev->out_ep_maxpsize = max_t(u16,
				le16_to_cpu(fs_epout_desc.wMaxPacketSize),
				le16_to_cpu(hs_epout_desc.wMaxPacketSize));

	agdev->in_ep_maxpsize = max_t(u16, agdev->in_ep_maxpsize,
				le16_to_cpu(ss_epin_desc.wMaxPacketSize));
	agdev->out_ep_maxpsize = max_t(u16, agdev->out_ep_maxpsize,
				le16_to_cpu(ss_epout_desc.wMaxPacketSize));

	ss_epin_desc_comp.wBytesPerInterval = ss_epin_desc.wMaxPacketSize;
	ss_epout_desc_comp.wBytesPerInterval = ss_epout_desc.wMaxPacketSize;

	// HS and SS endpoint addresses are copied from autoconfigured FS descriptors
	hs_epout_desc.bEndpointAddress = fs_epout_desc.bEndpointAddress;
	hs_epin_fback_desc.bEndpointAddress = fs_epin_fback_desc.bEndpointAddress;
	hs_epin_desc.bEndpointAddress = fs_epin_desc.bEndpointAddress;
	ss_epout_desc.bEndpointAddress = fs_epout_desc.bEndpointAddress;
	ss_epin_fback_desc.bEndpointAddress = fs_epin_fback_desc.bEndpointAddress;
	ss_epin_desc.bEndpointAddress = fs_epin_desc.bEndpointAddress;

	setup_descriptor(uac2_opts);

	ret = usb_assign_descriptors(fn, fs_audio_desc, hs_audio_desc, ss_audio_desc,
				     ss_audio_desc);
	if (ret)
		goto err_free_fu;

	agdev->gadget = gadget;

	agdev->params.p_chmask = uac2_opts->p_chmask;
	agdev->params.c_chmask = uac2_opts->c_chmask;
	memcpy(agdev->params.srates, uac2_opts->srates,
		   sizeof(agdev->params.srates));
	agdev->params.ssize = uac2_opts->ssize;
	agdev->params.req_number = uac2_opts->req_number;
	agdev->params.fb_max = uac2_opts->fb_max;

	ret = g_audio_setup(agdev, "UAC2 PCM", "UAC2_Gadget");
	if (ret)
		goto err_free_descs;

	return 0;

err_free_descs:
	usb_free_all_descriptors(fn);
	agdev->gadget = NULL;
err_free_fu:
	return ret;
}

static int
afunc_set_alt(struct usb_function *fn, unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct f_uac2 *uac2 = func_to_uac2(fn);
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &gadget->dev;
	int ret = 0;

	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (intf == uac2->ac_intf) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}

		return 0;
	}

	if (intf == uac2->as_out_intf) {
		uac2->as_out_alt = alt;

		if (alt)
			ret = u_audio_start_capture(&uac2->g_audio);
		else
			u_audio_stop_capture(&uac2->g_audio);
	} else if (intf == uac2->as_in_intf) {
		uac2->as_in_alt = alt;

		if (alt)
			ret = u_audio_start_playback(&uac2->g_audio);
		else
			u_audio_stop_playback(&uac2->g_audio);
	} else {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	return ret;
}

static int
afunc_get_alt(struct usb_function *fn, unsigned intf)
{
	struct f_uac2 *uac2 = func_to_uac2(fn);
	struct g_audio *agdev = func_to_g_audio(fn);

	if (intf == uac2->ac_intf)
		return uac2->ac_alt;
	else if (intf == uac2->as_out_intf)
		return uac2->as_out_alt;
	else if (intf == uac2->as_in_intf)
		return uac2->as_in_alt;
	else
		dev_err(&agdev->gadget->dev,
			"%s:%d Invalid Interface %d!\n",
			__func__, __LINE__, intf);

	return -EINVAL;
}

static void
afunc_disable(struct usb_function *fn)
{
	struct f_uac2 *uac2 = func_to_uac2(fn);

	uac2->as_in_alt = 0;
	uac2->as_out_alt = 0;
	u_audio_stop_capture(&uac2->g_audio);
	u_audio_stop_playback(&uac2->g_audio);
}

static void
afunc_suspend(struct usb_function *fn)
{
	struct f_uac2 *uac2 = func_to_uac2(fn);

	u_audio_suspend(&uac2->g_audio);
}

static int
in_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct g_audio *agdev = func_to_g_audio(fn);
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;
	u32 srate;

	u_audio_get_srate(agdev, &srate);

	if (entity_id == USB_CLK_ID) {
		if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
			struct cntrl_cur_lay3 c;

			memset(&c, 0, sizeof(struct cntrl_cur_lay3));

			c.dCUR = cpu_to_le32(srate);

			value = min_t(unsigned int, w_length, sizeof(c));
			memcpy(req->buf, &c, value);
		} else if (control_selector == UAC2_CS_CONTROL_CLOCK_VALID) {
			*(u8 *)req->buf = 1;
			value = min_t(unsigned int, w_length, 1);
		} else {
			dev_err(&agdev->gadget->dev,
				"%s:%d control_selector=%d TODO!\n",
				__func__, __LINE__, control_selector);
		}
	} else {
		dev_err(&agdev->gadget->dev,
			"%s:%d entity_id=%d control_selector=%d TODO!\n",
			__func__, __LINE__, entity_id, control_selector);
	}

	return value;
}

static int
in_rq_range(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct g_audio *agdev = func_to_g_audio(fn);
	struct f_uac2_opts *opts = g_audio_to_uac2_opts(agdev);
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;

	if (entity_id == USB_CLK_ID) {
		if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
			struct cntrl_ranges_lay3_srates rs;
			int i;
			int wNumSubRanges = 0;
			int srate;
			int *srates;

			srates = opts->srates;

			for (i = 0; i < UAC_MAX_RATES; i++) {
				srate = srates[i];
				if (srate == 0)
					break;

				rs.r[wNumSubRanges].dMIN = cpu_to_le32(srate);
				rs.r[wNumSubRanges].dMAX = cpu_to_le32(srate);
				rs.r[wNumSubRanges].dRES = 0;
				wNumSubRanges++;
				dev_dbg(&agdev->gadget->dev,
					"%s(): clk %d: rate ID %d: %d\n",
					__func__, entity_id, wNumSubRanges, srate);
			}
			rs.wNumSubRanges = cpu_to_le16(wNumSubRanges);
			value = min_t(unsigned int, w_length, ranges_lay3_size(rs));
			dev_dbg(&agdev->gadget->dev, "%s(): sending %d rates, size %d\n",
				__func__, rs.wNumSubRanges, value);
			memcpy(req->buf, &rs, value);
		} else {
			dev_err(&agdev->gadget->dev,
				"%s:%d control_selector=%d TODO!\n",
				__func__, __LINE__, control_selector);
		}
	} else {
		dev_err(&agdev->gadget->dev,
			"%s:%d entity_id=%d control_selector=%d TODO!\n",
			__func__, __LINE__, entity_id, control_selector);
	}

	return value;
}

static int
ac_rq_in(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	if (cr->bRequest == UAC2_CS_CUR)
		return in_rq_cur(fn, cr);
	else if (cr->bRequest == UAC2_CS_RANGE)
		return in_rq_range(fn, cr);
	else
		return -EOPNOTSUPP;
}

static void uac2_cs_control_sam_freq(struct usb_ep *ep, struct usb_request *req)
{
	struct usb_function *fn = ep->driver_data;
	struct g_audio *agdev = func_to_g_audio(fn);
	struct f_uac2 *uac2 = func_to_uac2(fn);
	u32 val;

	if (req->actual != 4)
		return;

	val = le32_to_cpu(*((__le32 *)req->buf));
	dev_dbg(&agdev->gadget->dev, "%s val: %d.\n", __func__, val);
	if (uac2->clock_id == USB_CLK_ID) {
		u_audio_set_srate(agdev, val);
	}
}

static int
out_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct usb_request *req = fn->config->cdev->req;
	struct g_audio *agdev = func_to_g_audio(fn);
	struct f_uac2 *uac2 = func_to_uac2(fn);
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	u8 clock_id = w_index >> 8;

	if (entity_id == USB_CLK_ID) {
		if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
			dev_dbg(&agdev->gadget->dev,
				"control_selector UAC2_CS_CONTROL_SAM_FREQ, clock: %d\n", clock_id);
			cdev->gadget->ep0->driver_data = fn;
			uac2->clock_id = clock_id;
			req->complete = uac2_cs_control_sam_freq;
			return w_length;
		}
	} else {
		dev_err(&agdev->gadget->dev,
			"%s:%d entity_id=%d control_selector=%d TODO!\n",
			__func__, __LINE__, entity_id, control_selector);
	}
	return -EOPNOTSUPP;
}

static int
setup_rq_inf(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct f_uac2 *uac2 = func_to_uac2(fn);
	struct g_audio *agdev = func_to_g_audio(fn);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u8 intf = w_index & 0xff;

	if (intf != uac2->ac_intf) {
		dev_err(&agdev->gadget->dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EOPNOTSUPP;
	}

	if (cr->bRequestType & USB_DIR_IN)
		return ac_rq_in(fn, cr);
	else if (cr->bRequest == UAC2_CS_CUR)
		return out_rq_cur(fn, cr);

	return -EOPNOTSUPP;
}

static int
afunc_setup(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct g_audio *agdev = func_to_g_audio(fn);
	struct usb_request *req = cdev->req;
	u16 w_length = le16_to_cpu(cr->wLength);
	int value = -EOPNOTSUPP;

	/* Only Class specific requests are supposed to reach here */
	if ((cr->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS)
		return -EOPNOTSUPP;

	if ((cr->bRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE)
		value = setup_rq_inf(fn, cr);
	else
		dev_err(&agdev->gadget->dev, "%s:%d Error!\n",
				__func__, __LINE__);

	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			dev_err(&agdev->gadget->dev,
				"%s:%d Error!\n", __func__, __LINE__);
			req->status = 0;
		}
	}

	return value;
}

static inline struct f_uac2_opts *to_f_uac2_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_uac2_opts,
			    func_inst.group);
}

static void f_uac2_attr_release(struct config_item *item)
{
	struct f_uac2_opts *opts = to_f_uac2_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations f_uac2_item_ops = {
	.release	= f_uac2_attr_release,
};

#define uac2_kstrtou8 kstrtou8
#define uac2_kstrtou32 kstrtou32
#define uac2_kstrtos16 kstrtos16
#define uac2_kstrtobool(s, base, res) kstrtobool((s), (res))

static const char *u8_fmt = "%u\n";
static const char *u32_fmt = "%u\n";

#define UAC2_ATTRIBUTE(type, name)					\
static ssize_t f_uac2_opts_##name##_show(struct config_item *item,	\
					 char *page)			\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int result;							\
									\
	mutex_lock(&opts->lock);					\
	result = sprintf(page, type##_fmt, opts->name);			\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac2_opts_##name##_store(struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int ret;							\
	type num;							\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	ret = uac2_kstrto##type(page, 0, &num);				\
	if (ret)							\
		goto end;						\
									\
	opts->name = num;						\
	ret = len;							\
									\
end:									\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac2_opts_, name)

#define UAC2_ATTRIBUTE_SYNC(name)					\
static ssize_t f_uac2_opts_##name##_show(struct config_item *item,	\
					 char *page)			\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int result;							\
	char *str;							\
									\
	mutex_lock(&opts->lock);					\
	switch (opts->name) {						\
	case USB_ENDPOINT_SYNC_ASYNC:					\
		str = "async";						\
		break;							\
	case USB_ENDPOINT_SYNC_ADAPTIVE:				\
		str = "adaptive";					\
		break;							\
	default:							\
		str = "unknown";					\
		break;							\
	}								\
	result = sprintf(page, "%s\n", str);				\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac2_opts_##name##_store(struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int ret = 0;							\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	if (!strncmp(page, "async", 5))					\
		opts->name = USB_ENDPOINT_SYNC_ASYNC;			\
	else if (!strncmp(page, "adaptive", 8))				\
		opts->name = USB_ENDPOINT_SYNC_ADAPTIVE;		\
	else {								\
		ret = -EINVAL;						\
		goto end;						\
	}								\
									\
	ret = len;							\
									\
end:									\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac2_opts_, name)

#define UAC2_RATE_ATTRIBUTE(name)					\
static ssize_t f_uac2_opts_##name##_show(struct config_item *item,	\
					 char *page)			\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int result = 0;							\
	int i;								\
									\
	mutex_lock(&opts->lock);					\
	page[0] = '\0';							\
	for (i = 0; i < UAC_MAX_RATES; i++) {				\
		if (opts->name##s[i] == 0)				\
			break;						\
		result += sprintf(page + strlen(page), "%u,",		\
				opts->name##s[i]);			\
	}								\
	if (strlen(page) > 0)						\
		page[strlen(page) - 1] = '\n';				\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac2_opts_##name##_store(struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	char *split_page = NULL;					\
	int ret = -EINVAL;						\
	char *token;							\
	u32 num;							\
	int i;								\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	i = 0;								\
	memset(opts->name##s, 0x00, sizeof(opts->name##s));		\
	split_page = kstrdup(page, GFP_KERNEL);				\
	while ((token = strsep(&split_page, ",")) != NULL) {		\
		ret = kstrtou32(token, 0, &num);			\
		if (ret)						\
			goto end;					\
									\
		opts->name##s[i++] = num;				\
		ret = len;						\
	};								\
									\
end:									\
	kfree(split_page);						\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac2_opts_, name)

#define UAC2_ATTRIBUTE_STRING(name)					\
static ssize_t f_uac2_opts_##name##_show(struct config_item *item,	\
					 char *page)			\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int result;							\
									\
	mutex_lock(&opts->lock);					\
	result = snprintf(page, sizeof(opts->name), "%s", opts->name);	\
	mutex_unlock(&opts->lock);					\
									\
	return result;							\
}									\
									\
static ssize_t f_uac2_opts_##name##_store(struct config_item *item,	\
					  const char *page, size_t len)	\
{									\
	struct f_uac2_opts *opts = to_f_uac2_opts(item);		\
	int ret = 0;							\
									\
	mutex_lock(&opts->lock);					\
	if (opts->refcnt) {						\
		ret = -EBUSY;						\
		goto end;						\
	}								\
									\
	ret = snprintf(opts->name, min(sizeof(opts->name), len),	\
			"%s", page);					\
									\
end:									\
	mutex_unlock(&opts->lock);					\
	return ret;							\
}									\
									\
CONFIGFS_ATTR(f_uac2_opts_, name)

UAC2_ATTRIBUTE(u32, p_chmask);
UAC2_ATTRIBUTE(u8, p_hs_bint);
UAC2_ATTRIBUTE(u32, c_chmask);
UAC2_ATTRIBUTE_SYNC(c_sync);
UAC2_ATTRIBUTE(u8, c_hs_bint);
UAC2_RATE_ATTRIBUTE(srate);
UAC2_ATTRIBUTE(u32, ssize);
UAC2_ATTRIBUTE(u32, req_number);

UAC2_ATTRIBUTE(u32, fb_max);
UAC2_ATTRIBUTE_STRING(function_name);

static struct configfs_attribute *f_uac2_attrs[] = {
	&f_uac2_opts_attr_p_chmask,
	&f_uac2_opts_attr_p_hs_bint,
	&f_uac2_opts_attr_c_chmask,
	&f_uac2_opts_attr_c_hs_bint,
	&f_uac2_opts_attr_c_sync,
	&f_uac2_opts_attr_srate,
	&f_uac2_opts_attr_ssize,
	&f_uac2_opts_attr_req_number,
	&f_uac2_opts_attr_fb_max,
	&f_uac2_opts_attr_function_name,
	NULL,
};

static const struct config_item_type f_uac2_func_type = {
	.ct_item_ops	= &f_uac2_item_ops,
	.ct_attrs	= f_uac2_attrs,
	.ct_owner	= THIS_MODULE,
};

static void afunc_free_inst(struct usb_function_instance *f)
{
	struct f_uac2_opts *opts;

	opts = container_of(f, struct f_uac2_opts, func_inst);
	kfree(opts);
}

static struct usb_function_instance *afunc_alloc_inst(void)
{
	struct f_uac2_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = afunc_free_inst;

	config_group_init_type_name(&opts->func_inst.group, "",
				    &f_uac2_func_type);

	opts->p_chmask = UAC2_DEF_PCHMASK;
	opts->p_hs_bint = UAC2_DEF_PHSBINT;
	opts->c_chmask = UAC2_DEF_CCHMASK;
	opts->c_hs_bint = UAC2_DEF_CHSBINT;
	opts->c_sync = UAC2_DEF_CSYNC;

	opts->srates[0] = UAC2_DEF_SRATE;
	opts->ssize = UAC2_DEF_SSIZE;

	opts->req_number = UAC2_DEF_REQ_NUM;
	opts->fb_max = FBACK_FAST_MAX;

	snprintf(opts->function_name, sizeof(opts->function_name), "Source/Sink");

	return &opts->func_inst;
}

static void afunc_free(struct usb_function *f)
{
	struct g_audio *agdev;
	struct f_uac2_opts *opts;

	agdev = func_to_g_audio(f);
	opts = container_of(f->fi, struct f_uac2_opts, func_inst);
	kfree(agdev);
	mutex_lock(&opts->lock);
	--opts->refcnt;
	mutex_unlock(&opts->lock);
}

static void afunc_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct g_audio *agdev = func_to_g_audio(f);

	g_audio_cleanup(agdev);
	usb_free_all_descriptors(f);

	agdev->gadget = NULL;
}

static struct usb_function *afunc_alloc(struct usb_function_instance *fi)
{
	struct f_uac2	*uac2;
	struct f_uac2_opts *opts;

	uac2 = kzalloc(sizeof(*uac2), GFP_KERNEL);
	if (uac2 == NULL)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_uac2_opts, func_inst);
	mutex_lock(&opts->lock);
	++opts->refcnt;
	mutex_unlock(&opts->lock);

	uac2->g_audio.func.name = "uac2_func";
	uac2->g_audio.func.bind = afunc_bind;
	uac2->g_audio.func.unbind = afunc_unbind;
	uac2->g_audio.func.set_alt = afunc_set_alt;
	uac2->g_audio.func.get_alt = afunc_get_alt;
	uac2->g_audio.func.disable = afunc_disable;
	uac2->g_audio.func.suspend = afunc_suspend;
	uac2->g_audio.func.setup = afunc_setup;
	uac2->g_audio.func.free_func = afunc_free;

	return &uac2->g_audio.func;
}

DECLARE_USB_FUNCTION_INIT(uac2, afunc_alloc_inst, afunc_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yadwinder Singh");
MODULE_AUTHOR("Jaswinder Singh");
MODULE_AUTHOR("Ruslan Bilovol");
