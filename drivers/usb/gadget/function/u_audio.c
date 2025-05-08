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
#include <linux/proc_fs.h>
#include <linux/usb/audio.h>

#include "u_audio.h"

#define CLK_PPM_GROUP_SIZE	10

/* incremented on i2s side for keeping sync */
uint64_t uac_sync_samples = 0;

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

	struct usb_request **reqs;

	struct usb_request *req_fback; /* Feedback endpoint request */
	bool fb_ep_enabled; /* if the ep is enabled */

	struct snd_kcontrol *snd_kctl_rate; /* read-only current rate */
	bool active; /* playback/capture running */
	bool playback; /* capture if false */

	spinlock_t lock; /* lock for control transfers */
};

struct snd_uac_chip {
	struct g_audio *audio_dev;

	struct uac_rtd_params p_prm;
	struct uac_rtd_params c_prm;

	int srate; /* selected samplerate */
	bool fb_received; /* whether feedback ep actually works */

	/* pre-calculated values for playback iso completion */
	unsigned long long p_residue_mil;
	unsigned int p_interval;
	unsigned int p_framesize;
};

static struct snd_uac_chip *_uac;
static struct class *audio_class;

static void u_audio_set_fback_frequency(enum usb_device_speed speed,
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
}

static void u_audio_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned int pending;
	unsigned int hw_ptr;
	int status = req->status;
	struct uac_rtd_params *prm = req->context;
	struct uac_mmap_data *mdata = prm->mdata;
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	unsigned int frames, p_pktsize;
	unsigned long long pitched_rate_mil, p_pktsize_residue_mil,
			residue_frames_mil, div_result;

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

	if (prm->playback) {
		/*
		 * For each IN packet, take the quotient of the current data
		 * rate and the endpoint's interval as the base packet size.
		 * If there is a residue from this division, add it to the
		 * residue accumulator.
		 */
		unsigned long long p_interval_mil = uac->p_interval * 1000000ULL;

		if (uac->fb_received)
			pitched_rate_mil = (unsigned long long) uac->srate * 1000000;
		else
			pitched_rate_mil = (unsigned long long) uac->srate * (1000000 - audio_dev->params.ppm + prm->mdata->extra_ppm);
		div_result = pitched_rate_mil;
		do_div(div_result, uac->p_interval);
		do_div(div_result, 1000000);
		frames = (unsigned int) div_result;

		pr_debug("srate %d, pitch %d, interval_mil %llu, frames %d\n",
				uac->srate,
				1000000 + audio_dev->params.ppm + prm->mdata->extra_ppm,
				p_interval_mil,
				frames);

		p_pktsize = min_t(unsigned int,
					uac->p_framesize * frames,
					ep->maxpacket);

		if (p_pktsize < ep->maxpacket) {
			residue_frames_mil = pitched_rate_mil - frames * p_interval_mil;
			p_pktsize_residue_mil = uac->p_framesize * residue_frames_mil;
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
		if ((unsigned int) div_result >= uac->p_framesize) {
			req->length += uac->p_framesize;
			uac->p_residue_mil -= uac->p_framesize * p_interval_mil;
			pr_debug("increased req length to %d\n", req->length);
		}
		pr_debug("remains uac->p_residue_mil %llu\n", uac->p_residue_mil);

		req->actual = req->length;
	}

	__atomic_load_n(&mdata->bufpos_userspace, __ATOMIC_ACQUIRE);
	hw_ptr = mdata->bufpos_kernel;

	/* Pack USB load in mmap ring buffer */
	pending = mdata->buffer_size - hw_ptr;

	if (prm->playback) {
		if (unlikely(pending < req->actual)) {
			memcpy(req->buf, mdata->buffer + hw_ptr, pending);
			memcpy(req->buf + pending, mdata->buffer, req->actual - pending);
		} else {
			memcpy(req->buf, mdata->buffer + hw_ptr, req->actual);
		}
	} else {
		if (unlikely(pending < req->actual)) {
			memcpy(mdata->buffer + hw_ptr, req->buf, pending);
			memcpy(mdata->buffer, req->buf + pending, req->actual - pending);
		} else {
			memcpy(mdata->buffer + hw_ptr, req->buf, req->actual);
		}
	}

	/* update hw_ptr after data is copied to memory */
	hw_ptr = (hw_ptr + req->actual) % mdata->buffer_size;
	__atomic_store_n(&mdata->bufpos_kernel, hw_ptr, __ATOMIC_RELEASE);

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
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);
	else
		uac->fb_received = true;

	u_audio_set_fback_frequency(audio_dev->gadget->speed, audio_dev->out_ep,
				    uac->srate, 1000000 - audio_dev->params.ppm + prm->mdata->extra_ppm,
				    req->buf);

	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(audio_dev->device, "%d Error!\n", __LINE__);
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
	struct g_audio *audio_dev = uac->audio_dev;

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
		dev_err(audio_dev->device, "%s:%d Error!\n", __func__, __LINE__);
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
	unsigned long c_flags, p_flags;

	dev_dbg(&audio_dev->gadget->dev, "%s: srate %d\n", __func__, srate);
	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (params->srates[i] == srate) {
			spin_lock_irqsave(&uac->c_prm.lock, c_flags);
			spin_lock_irqsave(&uac->p_prm.lock, p_flags);
			uac->srate = srate;
			uac->c_prm.mdata->sample_rate = srate;
			uac->p_prm.mdata->sample_rate = srate;
			audio_dev->usb_state[SET_SAMPLE_RATE] = true;
			schedule_work(&audio_dev->work);
			spin_unlock_irqrestore(&uac->p_prm.lock, p_flags);
			spin_unlock_irqrestore(&uac->c_prm.lock, c_flags);
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
	unsigned long c_flags, p_flags;

	spin_lock_irqsave(&uac->c_prm.lock, c_flags);
	spin_lock_irqsave(&uac->p_prm.lock, p_flags);
	*val = uac->srate;
	spin_unlock_irqrestore(&uac->p_prm.lock, p_flags);
	spin_unlock_irqrestore(&uac->c_prm.lock, c_flags);
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
	int req_len, i;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * capture last time. It needs to stop capture
	 * prior to start capture next time.
	 */
	if (audio_dev->stream_state[STATE_OUT])
		u_audio_stop_capture(audio_dev);

	audio_dev->usb_state[SET_INTERFACE_OUT] = true;
	audio_dev->stream_state[STATE_OUT] = true;
	schedule_work(&audio_dev->work);

	prm = &uac->c_prm;
	dev_dbg(dev, "start capture with rate %d\n", uac->srate);
	ep = audio_dev->out_ep;
	config_ep_by_speed(gadget, &audio_dev->func, ep);
	req_len = ep->maxpacket;

	/* check if we receive feedback requests */
	uac->fb_received = false;

	prm->ep_enabled = true;
	usb_ep_enable(ep);

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
	config_ep_by_speed(gadget, &audio_dev->func, ep_fback);
	prm->fb_ep_enabled = true;
	usb_ep_enable(ep_fback);
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

	audio_dev->usb_state[SET_INTERFACE_OUT] = true;
	audio_dev->stream_state[STATE_OUT] = false;
	schedule_work(&audio_dev->work);
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
	int req_len, i;
	unsigned int p_pktsize;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * playback last time. It needs to stop playback
	 * prior to start playback next time.
	 */
	if (audio_dev->stream_state[STATE_IN])
		u_audio_stop_playback(audio_dev);

	audio_dev->usb_state[SET_INTERFACE_IN] = true;
	audio_dev->stream_state[STATE_IN] = true;
	schedule_work(&audio_dev->work);

	prm = &uac->p_prm;
	dev_dbg(dev, "start playback with rate %d\n", uac->srate);
	ep = audio_dev->in_ep;
	config_ep_by_speed(gadget, &audio_dev->func, ep);

	ep_desc = ep->desc;

	/* pre-calculate the playback endpoint's interval */
	if (gadget->speed == USB_SPEED_FULL)
		factor = 1000;
	else
		factor = 8000;

	/* pre-compute some values for iso_complete() */
	uac->p_framesize = params->ssize *
			    num_channels(params->p_chmask);
	uac->p_interval = factor / (1 << (ep_desc->bInterval - 1));
	p_pktsize = min_t(unsigned int,
				uac->p_framesize *
					(uac->srate / uac->p_interval),
				ep->maxpacket);

	req_len = p_pktsize;
	uac->p_residue_mil = 0;

	prm->ep_enabled = true;
	usb_ep_enable(ep);

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

	audio_dev->usb_state[SET_INTERFACE_IN] = true;
	audio_dev->stream_state[STATE_IN] = false;
	schedule_work(&audio_dev->work);
}
EXPORT_SYMBOL_GPL(u_audio_stop_playback);

void u_audio_suspend(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->p_prm, false);
	set_active(&uac->c_prm, false);
}
EXPORT_SYMBOL_GPL(u_audio_suspend);

static void g_audio_work(struct work_struct *data)
{
	struct g_audio *audio = container_of(data, struct g_audio, work);
	struct usb_gadget *gadget = audio->gadget;
	struct snd_uac_chip *uac = audio->uac;
	struct device *dev = &gadget->dev;
	char *uac_event[4]  = { NULL, NULL, NULL, NULL };
	char str[19];
	int i;

	for (i = 0; i < SET_USB_STATE_MAX; i++) {
		if (!audio->usb_state[i])
			continue;

		switch (i) {
		case SET_INTERFACE_OUT:
			uac_event[0] = "USB_STATE=SET_INTERFACE";
			uac_event[1] = "STREAM_DIRECTION=OUT";
			uac_event[2] = audio->stream_state[STATE_OUT] ?
				       "STREAM_STATE=ON" : "STREAM_STATE=OFF";
			break;
		case SET_INTERFACE_IN:
			uac_event[0] = "USB_STATE=SET_INTERFACE";
			uac_event[1] = "STREAM_DIRECTION=IN";
			uac_event[2] = audio->stream_state[STATE_IN] ?
				       "STREAM_STATE=ON" : "STREAM_STATE=OFF";
			break;
		case SET_SAMPLE_RATE:
			uac_event[0] = "USB_STATE=SET_SAMPLE_RATE";
			snprintf(str, sizeof(str), "SAMPLE_RATE=%d",
				 uac->srate);
			uac_event[1] = str;
			break;
		case SET_AUDIO_CLK:
			uac_event[0] = "USB_STATE=SET_AUDIO_CLK";
			snprintf(str, sizeof(str), "PPM=%d", audio->params.ppm);
			uac_event[1] = str;
			break;
		default:
			break;
		}

		audio->usb_state[i] = false;
		kobject_uevent_env(&audio->device->kobj, KOBJ_CHANGE,
				   uac_event);
		dev_dbg(dev, "%s: sent uac uevent %s %s %s\n", __func__,
			uac_event[0], uac_event[1], uac_event[2]);
	}
}

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

	// time_now = ktime_get_raw();
	time_now = __atomic_load_n(&uac_sync_samples, __ATOMIC_SEQ_CST);

	if (time_now < uac->srate) {
		if (g_audio->fn->time_last) {
			memset(g_audio->fn, 0, sizeof(*g_audio->fn));
			g_audio->params.ppm = 0;
			g_audio->usb_state[SET_AUDIO_CLK] = true;
			schedule_work(&g_audio->work);
			// dev_warn(g_audio->device, "PPM is now reset\n");
		}
		goto out;
	}

	time_now = time_now * 1000000000ULL / uac->srate;
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
			g_audio->params.ppm = ppm;
			g_audio->usb_state[SET_AUDIO_CLK] = true;
			schedule_work(&g_audio->work);
			// dev_warn(g_audio->device, "PPM is now %d | extra_ppm %d\n", ppm, uac->c_prm.mdata->extra_ppm);
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

	if (c_chmask) {
		struct uac_rtd_params *prm = &uac->c_prm;

		spin_lock_init(&prm->lock);
		uac->c_prm.playback = false;
		uac->c_prm.uac = uac;
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

		channels = num_channels(c_chmask);
		buffer_size = 8192 * channels * params->ssize;
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

		spin_lock_init(&prm->lock);
		uac->p_prm.playback = true;
		uac->p_prm.uac = uac;
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

		channels = num_channels(p_chmask);
		buffer_size = 8192 * channels * params->ssize;
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

	g_audio->device = device_create(audio_class, NULL, MKDEV(0, 0), NULL, "uac2");
	if (IS_ERR(g_audio->device)) {
		err = PTR_ERR(g_audio->device);
		goto fail;
	}

	INIT_WORK(&g_audio->work, g_audio_work);
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

	cancel_work_sync(&g_audio->work);
	cancel_delayed_work_sync(&g_audio->ppm_work);
	device_destroy(g_audio->device->class, g_audio->device->devt);
	g_audio->device = NULL;

	uac = g_audio->uac;

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

	if (err == 0)
	{
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
