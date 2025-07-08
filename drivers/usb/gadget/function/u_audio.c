// SPDX-License-Identifier: GPL-2.0+
/*
 * u_audio.c -- interface to USB gadget "ALSA sound card" utilities
 *
 * Copyright (C) 2016
 * Author: Ruslan Bilovol <ruslan.bilovol@gmail.com>
 *
 * Sound card implementation was cut-and-pasted with changes
 * from f_uac2.c and has:
 *    Copyright (C) 2011
 *    Yadwinder Singh (yadi.brar01@gmail.com)
 *    Jaswinder Singh (jaswinder.singh@linaro.org)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb/audio.h>

#include <linux/highmem.h>
#include <linux/proc_fs.h>

#include "u_audio.h"

enum {
	UAC_FBACK_CTRL,
	UAC_P_PITCH_CTRL,
	UAC_MUTE_CTRL,
	UAC_VOLUME_CTRL,
	UAC_RATE_CTRL,
};

#define CLK_PPM_GROUP_SIZE	10

#if 0
/* incremented on i2s side for keeping sync */
uint64_t uac_sync_samples = 0;
#endif

/* shared mapped data */
struct uac_mmap_data {
	uint8_t active_kernel;
	uint8_t active_userspace;
	uint8_t data_size; // same as format
	uint8_t num_channels;
	uint32_t sample_rate;
	uint32_t buffer_size;
	uint32_t bufpos_kernel;
	uint32_t bufpos_userspace;
	int32_t extra_ppm;
	int32_t volume;
	uint8_t mute;
	uint8_t buffer[];
};

struct uac_proc_file {
	struct uac_mmap_data *mdata;
};

/* Runtime data params for one stream */
struct uac_rtd_params {
	struct snd_uac_chip *uac; /* parent chip */
	bool ep_enabled; /* if the ep is enabled */

	struct snd_pcm_substream *ss;

	/* Ring buffer */
	void *rbuf;
	struct uac_mmap_data* mdata;

	unsigned int max_psize;	/* MaxPacketSize of endpoint */
	unsigned int framesize;

	struct usb_request **reqs;

	struct usb_request *req_fback; /* Feedback endpoint request */
	bool fb_ep_enabled; /* if the ep is enabled */

	/* Volume/Mute controls and their state */
	s16 volume_min, volume_max, volume_res;

	bool active; /* playback/capture running */
	bool playback; /* capture if false */

	spinlock_t lock; /* lock for control transfers */
};

struct snd_uac_chip {
	struct g_audio *audio_dev;

	struct uac_rtd_params p_prm;
	struct uac_rtd_params c_prm;

	int srate; /* selected samplerate */
	uint64_t fb_received_time; /* time of last received feedback ep */

	/* pre-calculated values for playback iso completion */
	unsigned long long p_residue_mil;
	unsigned int p_interval;

	/* min/max buffer levels */
	int buf_p_min;
	int buf_p_max;
	int buf_c_min;
	int buf_c_max;

	/* min/max request sizes */
	int req_p_min;
	int req_p_max;
	int req_c_min;
	int req_c_max;

	/* number of requests adjusted in size */
	int adjusted_samples_up;
	int adjusted_samples_down;

	/* statistics counter */
	int fb_cnt;

	/* capture buffer state shared with playback stream */
	int out_buf_too_much;
	int out_buf_too_little;
};

static struct snd_uac_chip *_uac;
static struct class *audio_class;

static uint32_t u_audio_set_fback_frequency(enum usb_device_speed speed,
					struct usb_ep *out_ep,
					unsigned long long freq,
					unsigned int pitch,
					void *buf)
{
	u32 ff = 0;
	const struct usb_endpoint_descriptor *ep_desc;

	/*
	 * Because the pitch base is 1000000, the final divider here
	 * will be 1000 * 1000000 = 1953125 << 9
	 *
	 * Instead of dealing with big numbers lets fold this 9 left shift
	 */

	if (speed == USB_SPEED_FULL) {
		/*
		 * Full-speed feedback endpoints report frequency
		 * in samples/frame
		 * Format is encoded in Q10.10 left-justified in the 24 bits,
		 * so that it has a Q10.14 format.
		 *
		 * ff = (freq << 14) / 1000
		 */
		freq <<= 5;
	} else {
		/*
		 * High-speed feedback endpoints report frequency
		 * in samples/microframe.
		 * Format is encoded in Q12.13 fitted into four bytes so that
		 * the binary point is located between the second and the third
		 * byte fromat (that is Q16.16)
		 *
		 * ff = (freq << 16) / 8000
		 *
		 * Win10 and OSX UAC2 drivers require number of samples per packet
		 * in order to honor the feedback value.
		 * Linux snd-usb-audio detects the applied bit-shift automatically.
		 */
		ep_desc = out_ep->desc;
		freq <<= 4 + (ep_desc->bInterval - 1);
	}

	ff = DIV_ROUND_CLOSEST_ULL((freq * pitch), 1953125);

	*(__le32 *)buf = cpu_to_le32(ff);
	return ff;
}

static inline int32_t positive_modulo(int32_t i, int32_t n) {
    return (i % n + n) % n;
}

#define PLAYBACK_MIN_QUEUE (24)
#define PLAYBACK_MAX_QUEUE (80)
#define CAPTURE_MIN_QUEUE (16)
#define CAPTURE_MAX_QUEUE (96)


static void reset_stats(struct snd_uac_chip *uac)
{
	uac->buf_p_min = 65535;
	uac->buf_p_max = -65535;
	uac->buf_c_min = 65535;
	uac->buf_c_max = -65535;
	uac->req_p_min = 65535;
	uac->req_p_max = 0;
	uac->req_c_min = 65535;
	uac->req_c_max = 0;
	uac->adjusted_samples_up = 0;
	uac->adjusted_samples_down = 0;
	uac->fb_cnt = 0;
}

static void u_audio_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned int pending;
	unsigned int user_ptr;
	unsigned int hw_ptr;
	int status = req->status;
	struct uac_rtd_params *prm = req->context;
	struct uac_mmap_data *mdata = prm->mdata;
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	int ppm;
	unsigned int frames, p_pktsize;
	unsigned long long pitched_rate_mil, p_pktsize_residue_mil,
			residue_frames_mil, div_result;
	int buf_count;
	bool fb_ep_in_use;

	/* i/f shutting down */
	if (!prm->ep_enabled) {
		usb_ep_free_request(ep, req);
		return;
	}

	if (req->status == -ESHUTDOWN)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);

	// NOTE this assumes `bufpos_userspace` is right after `bufpos_kernel`
	invalidate_kernel_vmap_range(&mdata->bufpos_kernel, sizeof(uint32_t) * 2);

	hw_ptr = __atomic_load_n(&mdata->bufpos_kernel, __ATOMIC_ACQUIRE);
	user_ptr = __atomic_load_n(&mdata->bufpos_userspace, __ATOMIC_ACQUIRE);

	if (prm->playback) {
		/* safely check if feedback endpoint is in use */
		fb_ep_in_use = uac->c_prm.active
			&& uac->fb_received_time != 0
			&& ktime_get_raw() - uac->fb_received_time < 5000000 /* 5ms */;

		/*
		 * For each IN packet, take the quotient of the current data
		 * rate and the endpoint's interval as the base packet size.
		 * If there is a residue from this division, add it to the
		 * residue accumulator.
		 */
		unsigned long long p_interval_mil = uac->p_interval * 1000000ULL;

		ppm = - audio_dev->params.ppm;
		if (fb_ep_in_use)
			ppm += (prm->mdata->extra_ppm + uac->c_prm.mdata->extra_ppm) / 2;
		else
			ppm += prm->mdata->extra_ppm;

		pitched_rate_mil = (unsigned long long) uac->srate * (1000000 + ppm);
		div_result = pitched_rate_mil;
		do_div(div_result, uac->p_interval);
		do_div(div_result, 1000000);
		frames = (unsigned int) div_result;

		pr_debug("srate %d, pitch %d, interval_mil %llu, frames %d\n",
				uac->srate, 1000000 + ppm, p_interval_mil, frames);

		p_pktsize = min_t(unsigned int,
					prm->framesize * frames,
					ep->maxpacket);

		if (p_pktsize < ep->maxpacket) {
			residue_frames_mil = pitched_rate_mil - frames * p_interval_mil;
			p_pktsize_residue_mil = prm->framesize * residue_frames_mil;
		} else
			p_pktsize_residue_mil = 0;

		req->length = p_pktsize;
		uac->p_residue_mil += p_pktsize_residue_mil;

		/*
		 * Whenever there are more bytes in the accumulator p_residue_mil than we
		 * need to add one more sample frame, increase this packet's
		 * size and decrease the accumulator.
		 */
		div_result = uac->p_residue_mil;
		do_div(div_result, uac->p_interval);
		do_div(div_result, 1000000);
		if ((unsigned int) div_result >= prm->framesize) {
			req->length += prm->framesize;
			uac->p_residue_mil -= prm->framesize * p_interval_mil;
			pr_debug("increased req length to %d\n", req->length);
		}
		pr_debug("remains uac->p_residue_mil %llu\n", uac->p_residue_mil);

		buf_count = positive_modulo((int)user_ptr - (int)hw_ptr, mdata->buffer_size) / prm->framesize;

		if (!fb_ep_in_use) {
			int adjust_dir = 0;
			if (uac->out_buf_too_much) {
				adjust_dir = -1;
			}
			else if (buf_count > PLAYBACK_MAX_QUEUE) {
				dev_err(audio_dev->device, "playback queue above limit %d", buf_count);
				adjust_dir = 1;
			}
			else if (uac->out_buf_too_little) {
				adjust_dir = 1;
			}
			else if (buf_count > 0 && buf_count < PLAYBACK_MIN_QUEUE) {
				dev_err(audio_dev->device, "playback queue below limit %d", buf_count);
				adjust_dir = -1;
			}
			uac->out_buf_too_much = 0;
			uac->out_buf_too_little = 0;

			if (adjust_dir == 1 && req->length < prm->max_psize) {
				req->length += prm->framesize;
				//uac->p_residue_mil -= prm->framesize * p_interval_mil;
				uac->adjusted_samples_up++;
			}
			else if (adjust_dir == -1) {
				req->length -= prm->framesize;
				//uac->p_residue_mil += prm->framesize * p_interval_mil;
				uac->adjusted_samples_down--;
			}
		}

		req->actual = req->length;
	}
	else {
		buf_count = positive_modulo((int)hw_ptr - (int)user_ptr, mdata->buffer_size) / prm->framesize;
		if (buf_count > CAPTURE_MAX_QUEUE) {
			uac->out_buf_too_much = 1;
			dev_err(audio_dev->device, "capture queue above limit %d", buf_count);
		}
		else if (buf_count < CAPTURE_MIN_QUEUE) {
			uac->out_buf_too_little = 1;
			dev_err(audio_dev->device, "capture queue below limit %d", buf_count);
		}
	}

	/* Update statistics */
	if (prm->playback) {
		if (buf_count > uac->buf_p_max) {
			uac->buf_p_max = buf_count;
		}

		if (req->actual < uac->req_p_min) {
			uac->req_p_min = req->actual;
		}
		if (req->actual > uac->req_p_max) {
			uac->req_p_max = req->actual;
		}
	}
	else {
		if (buf_count < uac->buf_c_min) {
			uac->buf_c_min = buf_count;
		}

		if (req->actual < uac->req_c_min) {
			uac->req_c_min = req->actual;
		}
		if (req->actual > uac->req_c_max) {
			uac->req_c_max = req->actual;
		}
	}

	/* Pack USB load in mmap ring buffer */
	pending = mdata->buffer_size - hw_ptr;

	if (prm->playback) {
		if (unlikely(pending < req->actual)) {
			invalidate_kernel_vmap_range(mdata->buffer + hw_ptr, pending);
			invalidate_kernel_vmap_range(mdata->buffer, req->actual - pending);
			memcpy(req->buf, mdata->buffer + hw_ptr, pending);
			memcpy(req->buf + pending, mdata->buffer, req->actual - pending);
		} else {
			invalidate_kernel_vmap_range(mdata->buffer + hw_ptr, req->actual);
			memcpy(req->buf, mdata->buffer + hw_ptr, req->actual);
		}
	} else {
		if (unlikely(pending < req->actual)) {
			memcpy(mdata->buffer + hw_ptr, req->buf, pending);
			memcpy(mdata->buffer, req->buf + pending, req->actual - pending);
			flush_kernel_vmap_range(mdata->buffer + hw_ptr, pending);
			flush_kernel_vmap_range(mdata->buffer, req->actual - pending);
		} else {
			memcpy(mdata->buffer + hw_ptr, req->buf, req->actual);
			flush_kernel_vmap_range(mdata->buffer + hw_ptr, req->actual);
		}
	}

	if (prm->playback) {
		buf_count -= req->actual / prm->framesize;
		if (buf_count < uac->buf_p_min) {
			uac->buf_p_min = buf_count;
		}
	}
	else {
		buf_count += req->length / prm->framesize;
		if (buf_count > uac->buf_c_max) {
			uac->buf_c_max = buf_count;
		}
	}

	/* update hw_ptr after data is copied to memory */
	hw_ptr = (hw_ptr + req->actual) % mdata->buffer_size;
	__atomic_store_n(&mdata->bufpos_kernel, hw_ptr, __ATOMIC_RELEASE);

	flush_kernel_vmap_range(&mdata->bufpos_kernel, sizeof(uint32_t));

	if (prm->playback && !fb_ep_in_use) {
		uac->fb_cnt += req->length / prm->framesize;
		if (uac->fb_cnt >= mdata->sample_rate) {

			/* this is just copied from u_audio_set_fback_frequency */
			unsigned long long freq = uac->srate << 4;
			unsigned int pitch = 1000000 - audio_dev->params.ppm + prm->mdata->extra_ppm;
			uint32_t fb = DIV_ROUND_CLOSEST_ULL((freq * pitch), 1953125);

			printk(KERN_CRIT "uac NO FB %x\n", fb);
			printk(KERN_CRIT "uac buf play min %d max %d capture min %d max %d adj up %d down %d\n",
					uac->buf_p_min, uac->buf_p_max, uac->buf_c_min, uac->buf_c_max,
					uac->adjusted_samples_up, uac->adjusted_samples_down);
			printk(KERN_CRIT "uac req play min %d max %d capture min %d max %d\n",
					uac->req_p_min, uac->req_p_max, uac->req_c_min, uac->req_c_max);
			reset_stats(uac);
		}
	}

	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(audio_dev->device, "%d Error!\n", __LINE__);
}

static void u_audio_iso_fback_complete(struct usb_ep *ep,
				       struct usb_request *req)
{
	struct uac_rtd_params *prm = req->context;
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	int status = req->status;
	uint32_t fb;

	/* i/f shutting down */
	if (!prm->fb_ep_enabled) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
		return;
	}

	if (req->status == -ESHUTDOWN)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status) {
		uac->fb_received_time = 0;
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);
	} else {
		uac->fb_received_time = ktime_get_raw();
	}

	fb = u_audio_set_fback_frequency(audio_dev->gadget->speed, audio_dev->out_ep,
				    uac->srate, 1000000 - audio_dev->params.ppm + prm->mdata->extra_ppm,
				    req->buf);

	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(audio_dev->device, "%d Error!\n", __LINE__);

	uac->fb_cnt++;
	if (uac->fb_cnt >= 1024) {
		printk(KERN_CRIT "uac fb %x\n", fb);
		printk(KERN_CRIT "uac buf play min %d max %d capture min %d max %d\n",
				uac->buf_p_min, uac->buf_p_max, uac->buf_c_min, uac->buf_c_max);
		printk(KERN_CRIT "uac req play min %d max %d capture min %d max %d\n",
				uac->req_p_min, uac->req_p_max, uac->req_c_min, uac->req_c_max);
		reset_stats(uac);
	}
}

/* First page access. */
static vm_fault_t vm_fault(struct vm_fault *vmf)
{
	struct uac_proc_file *pfile;

	pfile = vmf->vma->vm_private_data;
	if (pfile->mdata) {
		vmf->page = vmalloc_to_page((uint8_t*)pfile->mdata + (vmf->pgoff << PAGE_SHIFT));
		get_page(vmf->page);
	}	return 0;
}

static struct vm_operations_struct vm_ops =
{
	.fault = vm_fault,
};

static int proc_open_c(struct inode *inode, struct file *file)
{
	struct uac_proc_file *pfile;

	if (_uac == NULL || _uac->c_prm.mdata == NULL)
		return -EINVAL;

	pfile = kmalloc(sizeof(struct uac_proc_file), GFP_KERNEL);
	pfile->mdata = _uac->c_prm.mdata;

	file->private_data = pfile;
	return 0;
}

static int proc_open_p(struct inode *inode, struct file *file)
{
	struct uac_proc_file *pfile;

	if (_uac == NULL || _uac->p_prm.mdata == NULL)
		return -EINVAL;

	pfile = kmalloc(sizeof(struct uac_proc_file), GFP_KERNEL);
	pfile->mdata = _uac->p_prm.mdata;

	file->private_data = pfile;
	return 0;
}

static int proc_release(struct inode *inode, struct file *file)
{
	struct uac_proc_file *pfile;

	pfile = file->private_data;
	kfree(pfile);

	file->private_data = NULL;
	return 0;
}

static int proc_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_ops = &vm_ops;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_private_data = file->private_data;
	return 0;
}

static ssize_t proc_read(struct file *file, char __user *buf, size_t len, loff_t *off)
{
	struct uac_proc_file *pfile;
	ssize_t ret;

	if ((size_t)*off >= PAGE_SIZE) {
		return 0;
	}

	pfile = file->private_data;
	ret = min(len, PAGE_SIZE - (size_t)*off);
	if (copy_to_user(buf, pfile->mdata + *off, ret)) {
		ret = -EFAULT;
	} else {
		*off += ret;
	}

	return ret;
}

static ssize_t proc_write(struct file *file, const char __user *buf, size_t len, loff_t *off)
{
	struct uac_proc_file *pfile;

	if ((size_t)*off >= PAGE_SIZE) {
		return 0;
	}

	pfile = file->private_data;
	if (copy_from_user(pfile->mdata + *off, buf, min(len, PAGE_SIZE - (size_t)*off))) {
		return -EFAULT;
	} else {
		return len;
	}
}

static const struct proc_ops proc_ops_c = {
	.proc_open = proc_open_c,
	.proc_release = proc_release,
	.proc_mmap = proc_mmap,
	.proc_read = proc_read,
	.proc_write = proc_write,
};

static const struct proc_ops proc_ops_p = {
	.proc_open = proc_open_p,
	.proc_release = proc_release,
	.proc_mmap = proc_mmap,
	.proc_read = proc_read,
	.proc_write = proc_write,
};

static inline void free_ep(struct uac_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev;
	struct uac_params *params;
	int i;

	if (!prm->ep_enabled)
		return;

	audio_dev = uac->audio_dev;
	params = &audio_dev->params;

	for (i = 0; i < params->req_number; i++) {
		if (prm->reqs[i]) {
			if (usb_ep_dequeue(ep, prm->reqs[i]))
				usb_ep_free_request(ep, prm->reqs[i]);
			/*
			 * If usb_ep_dequeue() cannot successfully dequeue the
			 * request, the request will be freed by the completion
			 * callback.
			 */

			prm->reqs[i] = NULL;
		}
	}

	prm->ep_enabled = false;

	if (usb_ep_disable(ep))
		dev_err(audio_dev->device, "%s:%d Error!\n", __func__, __LINE__);
}

static inline void free_ep_fback(struct uac_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_chip *uac = prm->uac;

	if (!prm->fb_ep_enabled)
		return;

	if (prm->req_fback) {
		if (usb_ep_dequeue(ep, prm->req_fback)) {
			kfree(prm->req_fback->buf);
			usb_ep_free_request(ep, prm->req_fback);
		}
		prm->req_fback = NULL;
	}

	prm->fb_ep_enabled = false;

	if (usb_ep_disable(ep))
		dev_err(uac->audio_dev->device, "%s:%d Error!\n", __func__, __LINE__);
}

static void set_active(struct uac_rtd_params *prm, bool active)
{
	unsigned long flags;

	spin_lock_irqsave(&prm->lock, flags);
	if (prm->active != active) {
		prm->active = active;
		prm->mdata->active_kernel = active;
		prm->mdata->bufpos_kernel = 0;
		prm->mdata->extra_ppm = 0;
		memset(prm->mdata->buffer, 0, prm->mdata->buffer_size);
	}
	spin_unlock_irqrestore(&prm->lock, flags);
}

int u_audio_set_srate(struct g_audio *audio_dev, int srate)
{
	struct uac_params *params = &audio_dev->params;
	struct snd_uac_chip *uac = audio_dev->uac;
	int i;
	unsigned long flags;

	dev_dbg(&audio_dev->gadget->dev, "%s: srate %d\n", __func__, srate);
	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (params->srates[i] == srate) {
			uac->srate = srate;

			spin_lock_irqsave(&uac->c_prm.lock, flags);
			uac->c_prm.mdata->sample_rate = srate;
			spin_unlock_irqrestore(&uac->c_prm.lock, flags);

			spin_lock_irqsave(&uac->p_prm.lock, flags);
			uac->p_prm.mdata->sample_rate = srate;
			spin_unlock_irqrestore(&uac->p_prm.lock, flags);
			return 0;
		}
		if (params->srates[i] == 0)
			break;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(u_audio_set_srate);

int u_audio_get_srate(struct g_audio *audio_dev, u32 *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	*val = uac->srate;
	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_srate);

int u_audio_start_capture(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct usb_gadget *gadget = audio_dev->gadget;
	struct device *dev = &gadget->dev;
	struct usb_request *req, *req_fback;
	struct usb_ep *ep, *ep_fback;
	struct uac_rtd_params *prm;
	struct uac_params *params = &audio_dev->params;
	int req_len, i, ret;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * capture last time. It needs to stop capture
	 * prior to start capture next time.
	 */
	if (audio_dev->stream_state[STATE_OUT])
		u_audio_stop_capture(audio_dev);

	audio_dev->stream_state[STATE_OUT] = true;

	prm = &uac->c_prm;
	dev_dbg(dev, "start capture with rate %d\n", uac->srate);
	ep = audio_dev->out_ep;
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed for out_ep failed (%d)\n", ret);
		return ret;
	}

	req_len = ep->maxpacket;

	/* check if we receive feedback requests */
	uac->fb_received_time = 0;

	prm->ep_enabled = true;
	ret = usb_ep_enable(ep);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for out_ep (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < params->req_number; i++) {
		if (!prm->reqs[i]) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->reqs[i] = req;

			req->zero = 0;
			req->context = prm;
			req->length = req_len;
			req->complete = u_audio_iso_complete;
			req->buf = prm->rbuf + i * ep->maxpacket;
		}

		if (usb_ep_queue(ep, prm->reqs[i], GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	set_active(&uac->c_prm, true);

	ep_fback = audio_dev->in_ep_fback;
	if (!ep_fback)
		return 0;

	/* Setup feedback endpoint */
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep_fback);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed in_ep_fback failed (%d)\n", ret);
		return ret; // TODO: Clean up out_ep
	}

	prm->fb_ep_enabled = true;
	ret = usb_ep_enable(ep_fback);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for in_ep_fback (%d)\n", ret);
		return ret; // TODO: Clean up out_ep
	}
	req_len = ep_fback->maxpacket;

	req_fback = usb_ep_alloc_request(ep_fback, GFP_ATOMIC);
	if (req_fback == NULL)
		return -ENOMEM;

	prm->req_fback = req_fback;
	req_fback->zero = 0;
	req_fback->context = prm;
	req_fback->length = req_len;
	req_fback->complete = u_audio_iso_fback_complete;

	req_fback->buf = kzalloc(req_len, GFP_ATOMIC);
	if (!req_fback->buf)
		return -ENOMEM;

	/*
	 * Configure the feedback endpoint's reported frequency.
	 * Always start with original frequency since its deviation can't
	 * be meauserd at start of playback
	 */
	u_audio_set_fback_frequency(audio_dev->gadget->speed, ep,
				    uac->srate, 1000000,
				    req_fback->buf);

	if (usb_ep_queue(ep_fback, req_fback, GFP_ATOMIC))
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_start_capture);

void u_audio_stop_capture(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->c_prm, false);
	if (audio_dev->in_ep_fback)
		free_ep_fback(&uac->c_prm, audio_dev->in_ep_fback);
	free_ep(&uac->c_prm, audio_dev->out_ep);

	audio_dev->stream_state[STATE_OUT] = false;
}
EXPORT_SYMBOL_GPL(u_audio_stop_capture);

int u_audio_start_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct usb_gadget *gadget = audio_dev->gadget;
	struct device *dev = &gadget->dev;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac_rtd_params *prm;
	struct uac_params *params = &audio_dev->params;
	unsigned int factor;
	const struct usb_endpoint_descriptor *ep_desc;
	int req_len, i, ret;
	unsigned int p_pktsize;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * playback last time. It needs to stop playback
	 * prior to start playback next time.
	 */
	if (audio_dev->stream_state[STATE_IN])
		u_audio_stop_playback(audio_dev);

	audio_dev->stream_state[STATE_IN] = true;

	prm = &uac->p_prm;
	dev_dbg(dev, "start playback with rate %d\n", uac->srate);
	ep = audio_dev->in_ep;
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed for in_ep failed (%d)\n", ret);
		return ret;
	}

	ep_desc = ep->desc;

	/* pre-calculate the playback endpoint's interval */
	if (gadget->speed == USB_SPEED_FULL)
		factor = 1000;
	else
		factor = 8000;

	/* pre-compute some values for iso_complete() */
	uac->p_interval = factor / (1 << (ep_desc->bInterval - 1));
	p_pktsize = min_t(unsigned int,
				prm->framesize *
					(uac->srate / uac->p_interval),
				ep->maxpacket);

	req_len = p_pktsize;
	uac->p_residue_mil = 0;

	prm->ep_enabled = true;
	ret = usb_ep_enable(ep);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for in_ep (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < params->req_number; i++) {
		if (!prm->reqs[i]) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->reqs[i] = req;

			req->zero = 0;
			req->context = prm;
			req->length = req_len;
			req->complete = u_audio_iso_complete;
			req->buf = prm->rbuf + i * ep->maxpacket;
		}

		if (usb_ep_queue(ep, prm->reqs[i], GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	set_active(&uac->p_prm, true);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_start_playback);

void u_audio_stop_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->p_prm, false);
	free_ep(&uac->p_prm, audio_dev->in_ep);

	audio_dev->stream_state[STATE_IN] = false;
}
EXPORT_SYMBOL_GPL(u_audio_stop_playback);

void u_audio_suspend(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->p_prm, false);
	set_active(&uac->c_prm, false);
}
EXPORT_SYMBOL_GPL(u_audio_suspend);

int u_audio_get_volume(struct g_audio *audio_dev, int playback, s16 *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->mdata->volume;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_volume);

int u_audio_set_volume(struct g_audio *audio_dev, int playback, s16 val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	val = clamp(val, prm->volume_min, prm->volume_max);
	prm->mdata->volume = val;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_set_volume);

int u_audio_get_mute(struct g_audio *audio_dev, int playback, int *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->mdata->mute;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_mute);

int u_audio_set_mute(struct g_audio *audio_dev, int playback, int val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;
	int mute;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	mute = val ? 1 : 0;

	spin_lock_irqsave(&prm->lock, flags);
	prm->mdata->mute = mute;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_set_mute);

static void ppm_calculate_work(struct work_struct *data)
{
	struct g_audio *g_audio = container_of(data, struct g_audio,
					       ppm_work.work);
	struct usb_gadget *gadget = g_audio->gadget;
	struct snd_uac_chip *uac = g_audio->uac;
	uint32_t frame_number, fn_msec, clk_msec;
	struct frame_number_data *fn = g_audio->fn;
	uint64_t time_now, time_msec_tmp;
	int32_t ppm;
	static int32_t ppms[CLK_PPM_GROUP_SIZE];
	static int32_t ppm_sum;
	int32_t cnt = fn->second % CLK_PPM_GROUP_SIZE;

#if 1
	time_now = ktime_get_raw();
#else
	time_now = __atomic_load_n(&uac_sync_samples, __ATOMIC_SEQ_CST);

	if (time_now < uac->srate) {
		if (g_audio->fn->time_last) {
			ppm_sum = 0;
			memset(ppms, 0, sizeof(ppms));
			memset(g_audio->fn, 0, sizeof(*g_audio->fn));
			g_audio->params.ppm = 0;
			// dev_warn(g_audio->device, "PPM is now reset\n");
		}
		// dev_warn(g_audio->device, "time_now < uac->srate\n");
		goto out;
	}

	time_now = time_now * 1000000000ULL / uac->srate;
#endif

	frame_number = gadget->ops->get_frame(gadget);

	if (g_audio->fn->time_last &&
	    time_now - g_audio->fn->time_last > 1500000000ULL)
		dev_warn(g_audio->device, "PPM work scheduled too slow!\n");

	g_audio->fn->time_last = time_now;

	/*
	 * If usb is disconnected, the controller will not receive the
	 * SoF signal and frame number will be invalid. Because we can't
	 * get accurate time of disconnect and whether the gadget will be
	 * plugged into the same host next time or not. We must clear all
	 * statistics.
	 */
	if (gadget->state != USB_STATE_CONFIGURED) {
		ppm_sum = 0;
		memset(ppms, 0, sizeof(ppms));
		memset(g_audio->fn, 0, sizeof(*g_audio->fn));
		dev_dbg(g_audio->device, "Disconnect. frame number is cleared\n");
		goto out;
	}

	/* Fist statistic to record begin frame number and system time */
	if (!g_audio->fn->second++) {
		g_audio->fn->time_begin = g_audio->fn->time_last;
		g_audio->fn->fn_begin = frame_number;
		g_audio->fn->fn_last = frame_number;
		goto out;
	}

	/*
	 * For DWC3 Controller, only 13 bits is used to store frame(micro)
	 * number. In other words, the frame number will overflow at most
	 * 2.047 seconds. We add another registor fn_overflow the record
	 * total frame number.
	 */
	if (frame_number <= g_audio->fn->fn_last)
		g_audio->fn->fn_overflow++;
	g_audio->fn->fn_last = frame_number;

	if (!g_audio->fn->fn_overflow)
		goto out;

	/* The lower 3 bits represent micro number frame, we don't need it */
	fn_msec = (((fn->fn_overflow - 1) << 14) +
		   (BIT(14) + fn->fn_last - fn->fn_begin) + BIT(2)) >> 3;
	time_msec_tmp = fn->time_last - fn->time_begin + 500000ULL;
	do_div(time_msec_tmp, 1000000U);
	clk_msec = (uint32_t)time_msec_tmp;

	/*
	 * According to the definition of ppm:
	 *   host_clk = (1 + ppm / 1000000) * gadget_clk
	 * we can get:
	 *   ppm = (host_clk - gadget_clk) * 1000000 / gadget_clk
	 */
	ppm = (fn_msec > clk_msec) ?
	      (fn_msec - clk_msec) * 1000000L / clk_msec :
	      -((clk_msec - fn_msec) * 1000000L / clk_msec);

	ppm_sum = ppm_sum - ppms[cnt] + ppm;
	ppms[cnt] = ppm;

	dev_dbg(g_audio->device,
		"frame %u msec %u ppm_calc %d ppm_avage(%d) %d\n",
		fn_msec, clk_msec, ppm, CLK_PPM_GROUP_SIZE,
		ppm_sum / CLK_PPM_GROUP_SIZE);

	/*
	 * We calculate the average of ppm over a period of time. If the
	 * latest frame number is too far from the average, no event will
	 * be sent.
	 */
	if (abs(ppm_sum / CLK_PPM_GROUP_SIZE - ppm) < 6) {
		ppm = ppm_sum > 0 ?
		      (ppm_sum + CLK_PPM_GROUP_SIZE / 2) / CLK_PPM_GROUP_SIZE :
		      (ppm_sum - CLK_PPM_GROUP_SIZE / 2) / CLK_PPM_GROUP_SIZE;
		if (ppm != g_audio->params.ppm) {
			if (abs(ppm) > 1000) {
				dev_warn(g_audio->device, "PPM reset abs(ppm) > 1000\n");
				g_audio->params.ppm = ppm_sum = ppm = 0;
				memset(ppms, 0, sizeof(ppms));
				memset(g_audio->fn, 0, sizeof(*g_audio->fn));
			}
			g_audio->params.ppm = ppm;
			// dev_warn(g_audio->device, "PPM is now %d | fb_received_time %llu | %llu | c extra_ppm %d | p extra_ppm %d\n",
			// 		ppm, uac->fb_received_time, ktime_get_raw() - uac->fb_received_time, uac->c_prm.mdata->extra_ppm, uac->p_prm.mdata->extra_ppm);
		}
	}

out:
	schedule_delayed_work(&g_audio->ppm_work, 1 * HZ);
}

int g_audio_setup(struct g_audio *g_audio, const char *pcm_name,
					const char *card_name)
{
	struct snd_uac_chip *uac;
	struct uac_params *params;
	uint32_t buffer_size, channels;
	int p_chmask, c_chmask;
	int err;

	if (!g_audio)
		return -EINVAL;

	uac = kzalloc(sizeof(*uac), GFP_KERNEL);
	if (!uac)
		return -ENOMEM;
	g_audio->uac = uac;
	uac->audio_dev = g_audio;

	params = &g_audio->params;
	p_chmask = params->p_chmask;
	c_chmask = params->c_chmask;

	uac->srate = params->srates[0];

	g_audio->fn = kzalloc(sizeof(*g_audio->fn), GFP_KERNEL);
	if (!g_audio->fn) {
		err = -ENOMEM;
		goto fail;
	}

	reset_stats(uac);

	if (c_chmask) {
		struct uac_rtd_params *prm = &uac->c_prm;
		channels = num_channels(c_chmask);

		spin_lock_init(&prm->lock);
		uac->c_prm.playback = false;
		uac->c_prm.uac = uac;
		prm->framesize = params->ssize * channels;
		prm->max_psize = g_audio->out_ep_maxpsize;

		prm->reqs = kcalloc(params->req_number,
				    sizeof(struct usb_request *),
				    GFP_KERNEL);
		if (!prm->reqs) {
			err = -ENOMEM;
			goto fail;
		}

		prm->rbuf = kcalloc(params->req_number, prm->max_psize,
				GFP_KERNEL);
		if (!prm->rbuf) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}

		buffer_size = 8192 * prm->framesize;
		prm->mdata = vmalloc_user(sizeof(struct uac_mmap_data) + buffer_size);
		if (!prm->mdata) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}

		prm->mdata->data_size = params->ssize;
		prm->mdata->num_channels = channels;
		prm->mdata->buffer_size = buffer_size;
		prm->mdata->sample_rate = uac->srate;
	}

	if (p_chmask) {
		struct uac_rtd_params *prm = &uac->p_prm;
		channels = num_channels(p_chmask);

		spin_lock_init(&prm->lock);
		uac->p_prm.playback = true;
		uac->p_prm.uac = uac;
		prm->framesize = params->ssize * channels;
		prm->max_psize = g_audio->in_ep_maxpsize;

		prm->reqs = kcalloc(params->req_number,
				    sizeof(struct usb_request *),
				    GFP_KERNEL);
		if (!prm->reqs) {
			err = -ENOMEM;
			goto fail;
		}

		prm->rbuf = kcalloc(params->req_number, prm->max_psize,
				GFP_KERNEL);
		if (!prm->rbuf) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}

		buffer_size = 8192 * prm->framesize;
		prm->mdata = vmalloc_user(sizeof(struct uac_mmap_data) + buffer_size);
		if (!prm->mdata) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}

		prm->mdata->data_size = params->ssize;
		prm->mdata->num_channels = channels;
		prm->mdata->buffer_size = buffer_size;
		prm->mdata->sample_rate = uac->srate;
	}

	g_audio->device = device_create(audio_class, NULL, MKDEV(0, 0), NULL,
					"uac2");
	if (IS_ERR(g_audio->device)) {
		err = PTR_ERR(g_audio->device);
		goto fail;
	}

	INIT_DELAYED_WORK(&g_audio->ppm_work, ppm_calculate_work);
	ppm_calculate_work(&g_audio->ppm_work.work);

	if (!err) {
		_uac = uac;
		return 0;
	}

fail:
	kfree(uac->p_prm.reqs);
	kfree(uac->c_prm.reqs);
	kfree(uac->p_prm.rbuf);
	kfree(uac->c_prm.rbuf);
	kfree(uac);
	kfree(g_audio->fn);

	return err;
}
EXPORT_SYMBOL_GPL(g_audio_setup);

void g_audio_cleanup(struct g_audio *g_audio)
{
	struct snd_uac_chip *uac;

	if (!g_audio || !g_audio->uac)
		return;

	_uac = NULL;

	cancel_delayed_work_sync(&g_audio->ppm_work);
	device_destroy(g_audio->device->class, g_audio->device->devt);
	g_audio->device = NULL;

	uac = g_audio->uac;
	g_audio->uac = NULL;

	kfree(uac->p_prm.reqs);
	kfree(uac->c_prm.reqs);
	kfree(uac->p_prm.rbuf);
	kfree(uac->c_prm.rbuf);
	kvfree(uac->c_prm.mdata);
	kvfree(uac->p_prm.mdata);
	kfree(uac);
	kfree(g_audio->fn);
}
EXPORT_SYMBOL_GPL(g_audio_cleanup);

static int __init u_audio_init(void)
{
	int err = 0;

	audio_class = class_create(THIS_MODULE, "u_audio");
	if (IS_ERR(audio_class)) {
		err = PTR_ERR(audio_class);
		audio_class = NULL;
	}

	if (err == 0) {
		proc_create("uac2c", 0, NULL, &proc_ops_c);
		proc_create("uac2p", 0, NULL, &proc_ops_p);
	}

	return err;
}
module_init(u_audio_init);

static void __exit u_audio_exit(void)
{
	if (audio_class)
		class_destroy(audio_class);
}
module_exit(u_audio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB gadget \"ALSA sound card\" utilities");
MODULE_AUTHOR("Ruslan Bilovol");
