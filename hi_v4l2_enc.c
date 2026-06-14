// SPDX-License-Identifier: GPL-2.0
/*
 * Hisilicon V4L2 memory-to-memory stateful video encoder.
 *
 * OUTPUT  queue: raw frames (NV12/NV21) -- vb2-dma-contig so the hardware gets a
 *                physically contiguous address for HI_DRV_VENC_QueueFrame().
 * CAPTURE queue: coded bitstream (H264/HEVC/JPEG).
 *
 * Backend: HI_DRV_VENC_* (venc_v2.0), via hi_venc_hal.c.
 *
 * PHASE1: synchronous 1-frame-in / 1-AU-out per device_run() (marked PHASE1).
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "hi_v4l2_fmt.h"
#include "hi_venc_hal.h"
#include "hi_v4l2_drv.h"

#define HI_ENC_NAME		"hi-venc"
#define HI_ENC_DEF_W		1280
#define HI_ENC_DEF_H		720
#define HI_ENC_MAX_W		4096
#define HI_ENC_MAX_H		4096
#define HI_ENC_MIN_CODED	(512 * 1024)
#define HI_ENC_DRAIN_EMPTY_MAX	8	/* empty drain polls before EOS */
#define HI_ENC_INFLIGHT_CAP	8	/* frames queued to VEDU before back-pressure */
#define HI_ENC_CAP_POLL_US_MIN	2000	/* AU poll while frames in flight */
#define HI_ENC_CAP_POLL_US_MAX	4000

static unsigned int debug;
module_param_named(enc_debug, debug, uint, 0644);
MODULE_PARM_DESC(enc_debug, "activates encoder debug info");

#define dprintk(d, fmt, arg...) \
	v4l2_dbg(1, debug, &(d)->v4l2_dev, "%s: " fmt, __func__, ## arg)

enum { Q_SRC = 0, Q_DST = 1 };

struct hi_enc_q_data {
	u32				width;
	u32				height;
	u32				sizeimage;
	u32				bytesperline;
	u32				sequence;
	const struct hi_v4l2_fmt	*fmt;
};

struct hi_enc_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct mutex		dev_mutex;
	struct v4l2_m2m_dev	*m2m_dev;
	void			*alloc_ctx;
	atomic_t		num_inst;
};

struct hi_enc_ctx {
	struct v4l2_fh		fh;
	struct hi_enc_dev	*dev;
	struct hi_venc_chan	*chan;
	struct v4l2_ctrl_handler hdl;
	struct hi_enc_q_data	q_data[2];
	struct hi_venc_params	params;
	bool			draining;	/* EOS drain (ENC_CMD_STOP) */
	int			drain_empty;	/* consecutive empty drain polls */
	/*
	 * T5-style decoupled encode (mirrors the decoder). device_run() only feeds
	 * raw frames to the VEDU (which reads them asynchronously) and parks them in
	 * an in-flight FIFO; a per-instance capture kthread pulls coded AUs out as
	 * they become ready, emits CAPTURE buffers, reclaims the VEDU input, and
	 * returns the matching OUTPUT (raw) buffer. The old synchronous feed-1/
	 * wait-1 model deadlocked the pipelined VEDU (no AU until >1 frame queued).
	 */
	struct task_struct	*cap_thread;
	wait_queue_head_t	cap_wq;
	bool			cap_streaming;	/* CAPTURE queue is streaming */
	spinlock_t		inflight_lock;
	struct vb2_v4l2_buffer	*inflight[HI_ENC_INFLIGHT_CAP];
	int			inflight_head, inflight_tail, inflight_cnt;
	/* colorimetry: set on OUTPUT, propagated to CAPTURE (V4L2 codec API) */
	u32			colorspace;
	u32			ycbcr_enc;
	u32			quantization;
	u32			xfer_func;
};

static inline struct hi_enc_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct hi_enc_ctx, fh);
}

static struct hi_enc_q_data *get_q_data(struct hi_enc_ctx *ctx,
					enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->q_data[Q_SRC]
					 : &ctx->q_data[Q_DST];
}

static u32 coded_sizeimage(u32 w, u32 h)
{
	return max(w * h, (u32)HI_ENC_MIN_CODED);
}

/* -------------------------------------------------------------------------- */
/* mem2mem ops                                                                */
/* -------------------------------------------------------------------------- */

/* in-flight FIFO of raw (OUTPUT) buffers queued to the VEDU, awaiting their AU */
static int hi_enc_inflight_cnt(struct hi_enc_ctx *ctx)
{
	unsigned long flags;
	int n;

	spin_lock_irqsave(&ctx->inflight_lock, flags);
	n = ctx->inflight_cnt;
	spin_unlock_irqrestore(&ctx->inflight_lock, flags);
	return n;
}

static void hi_enc_inflight_push(struct hi_enc_ctx *ctx,
				 struct vb2_v4l2_buffer *src)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->inflight_lock, flags);
	ctx->inflight[ctx->inflight_tail] = src;
	ctx->inflight_tail = (ctx->inflight_tail + 1) % HI_ENC_INFLIGHT_CAP;
	ctx->inflight_cnt++;
	spin_unlock_irqrestore(&ctx->inflight_lock, flags);
}

static struct vb2_v4l2_buffer *hi_enc_inflight_pop(struct hi_enc_ctx *ctx)
{
	struct vb2_v4l2_buffer *src = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ctx->inflight_lock, flags);
	if (ctx->inflight_cnt > 0) {
		src = ctx->inflight[ctx->inflight_head];
		ctx->inflight[ctx->inflight_head] = NULL;
		ctx->inflight_head = (ctx->inflight_head + 1) % HI_ENC_INFLIGHT_CAP;
		ctx->inflight_cnt--;
	}
	spin_unlock_irqrestore(&ctx->inflight_lock, flags);
	return src;
}

/*
 * device_run (T5): FEED-ONLY. Hands one raw frame to the VEDU (which reads it
 * asynchronously), parks it in the in-flight FIFO, and wakes the capture thread.
 * Back-pressure (job_ready) holds the feed at HI_ENC_INFLIGHT_CAP; the capture
 * thread re-arms the scheduler as it drains. Never blocks on coded output.
 */
static void device_run(void *priv)
{
	struct hi_enc_ctx *ctx = priv;
	struct hi_enc_dev *dev = ctx->dev;
	struct hi_enc_q_data *qs = &ctx->q_data[Q_SRC];
	struct vb2_v4l2_buffer *src;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	if (src && hi_enc_inflight_cnt(ctx) < HI_ENC_INFLIGHT_CAP) {
		dma_addr_t phys = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
		bool nv21 = (qs->fmt->fourcc == V4L2_PIX_FMT_NV21);
		u64 pts_ms = (u64)src->timestamp.tv_sec * 1000ULL +
			     src->timestamp.tv_usec / 1000;
		int ret = hi_venc_hal_queue_frame(ctx->chan, (u32)phys,
						  qs->bytesperline, qs->width,
						  qs->height, pts_ms, nv21);

		dprintk(dev, "queue_frame %ux%u ret=%d inflight=%d\n",
			qs->width, qs->height, ret, hi_enc_inflight_cnt(ctx) + 1);
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		src->sequence = qs->sequence++;
		hi_enc_inflight_push(ctx, src);
	}

	wake_up_interruptible(&ctx->cap_wq);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
}

/*
 * Capture thread: pulls coded AUs out of the VEDU as they become ready, emits
 * them as CAPTURE buffers, reclaims the VEDU input, and returns the matching
 * (FIFO) OUTPUT buffer. H264 streaming is 1-in/1-out (no reorder) so the oldest
 * in-flight frame owns each AU. Torn down with kthread_stop() at stop_streaming
 * BEFORE the buffer flush, so it never races the flush.
 */
static int hi_enc_cap_thread(void *data)
{
	struct hi_enc_ctx *ctx = data;
	struct hi_enc_dev *dev = ctx->dev;
	int drain_empty = 0;

	while (!kthread_should_stop()) {
		wait_event_interruptible(ctx->cap_wq,
			kthread_should_stop() ||
			(ctx->cap_streaming &&
			 (hi_enc_inflight_cnt(ctx) > 0 || ctx->draining)));
		if (kthread_should_stop())
			break;
		if (!ctx->cap_streaming)
			continue;

		drain_empty = 0;
		while (!kthread_should_stop() && ctx->cap_streaming &&
		       (hi_enc_inflight_cnt(ctx) > 0 || ctx->draining)) {
			struct hi_venc_stream st;
			struct vb2_v4l2_buffer *dst, *fsrc;

			if (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) < 1) {
				wait_event_interruptible_timeout(ctx->cap_wq,
					kthread_should_stop() || !ctx->cap_streaming ||
					v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) >= 1,
					msecs_to_jiffies(50));
				continue;
			}

			if (hi_venc_hal_acquire_stream(ctx->chan, &st) == 0) {
				dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
				fsrc = hi_enc_inflight_pop(ctx);
				if (dst) {
					u8 *o = vb2_plane_vaddr(&dst->vb2_buf, 0);
					u32 capsz = vb2_plane_size(&dst->vb2_buf, 0);
					u32 len = min(st.len, capsz);

					if (o && st.data)
						memcpy(o, st.data, len);
					vb2_set_plane_payload(&dst->vb2_buf, 0, len);
					dst->flags &= ~V4L2_BUF_FLAG_LAST;
					if (st.frame_end)
						dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
					dst->sequence = ctx->q_data[Q_DST].sequence++;
					if (fsrc)
						dst->timestamp = fsrc->timestamp;
				}
				hi_venc_hal_release_stream(ctx->chan, &st);
				hi_venc_hal_dequeue_frame(ctx->chan);
				if (fsrc)
					v4l2_m2m_buf_done(fsrc, VB2_BUF_STATE_DONE);
				if (dst)
					v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
				drain_empty = 0;
				v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
				continue;
			}

			/* No coded AU ready right now. */
			if (ctx->draining && hi_enc_inflight_cnt(ctx) == 0 &&
			    ++drain_empty >= HI_ENC_DRAIN_EMPTY_MAX) {
				dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
				if (dst) {
					static const struct v4l2_event ev = {
						.type = V4L2_EVENT_EOS };

					dprintk(dev, "drain complete -> LAST\n");
					vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
					dst->flags |= V4L2_BUF_FLAG_LAST;
					dst->sequence = ctx->q_data[Q_DST].sequence++;
					v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
					ctx->draining = false;
					v4l2_event_queue_fh(&ctx->fh, &ev);
				}
				break;
			}
			usleep_range(HI_ENC_CAP_POLL_US_MIN, HI_ENC_CAP_POLL_US_MAX);
		}
	}
	return 0;
}

static void hi_enc_cap_thread_start(struct hi_enc_ctx *ctx)
{
	if (ctx->cap_thread || !ctx->chan || !ctx->cap_streaming)
		return;
	ctx->cap_thread = kthread_run(hi_enc_cap_thread, ctx, "hi-venc-cap");
	if (IS_ERR(ctx->cap_thread)) {
		dprintk(ctx->dev, "cap_thread create failed (%ld)\n",
			PTR_ERR(ctx->cap_thread));
		ctx->cap_thread = NULL;
	}
}

static void hi_enc_cap_thread_stop(struct hi_enc_ctx *ctx)
{
	if (ctx->cap_thread) {
		kthread_stop(ctx->cap_thread);
		ctx->cap_thread = NULL;
	}
}

static int job_ready(void *priv)
{
	struct hi_enc_ctx *ctx = priv;

	/* Back-pressure: don't feed past the VEDU's in-flight window; the capture
	 * thread re-schedules as it drains. Draining is the kthread's job. */
	if (hi_enc_inflight_cnt(ctx) >= HI_ENC_INFLIGHT_CAP)
		return 0;
	return v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) >= 1;
}

static void job_abort(void *priv)
{
	/* Feed-only device_run() holds no blocking state; the capture thread is
	 * torn down at stop_streaming. Nothing to abort here. */
}

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_ready	= job_ready,
	.job_abort	= job_abort,
};

/* -------------------------------------------------------------------------- */
/* controls                                                                   */
/* -------------------------------------------------------------------------- */

static int hi_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hi_enc_ctx *ctx = container_of(ctrl->handler,
					      struct hi_enc_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->params.bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->params.gop = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
			ctx->params.profile = HI_UNF_H264_PROFILE_BASELINE; break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
			ctx->params.profile = HI_UNF_H264_PROFILE_HIGH; break;
		default:
			ctx->params.profile = HI_UNF_H264_PROFILE_MAIN; break;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		/* The VEDU encodes at its native level (4.1); accept the control
		 * so standard clients (GStreamer v4l2h264enc, ffmpeg h264_v4l2m2m,
		 * and thus tvheadend) can query profile+level during negotiation. */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops hi_enc_ctrl_ops = {
	.s_ctrl = hi_enc_s_ctrl,
};

/* -------------------------------------------------------------------------- */
/* ioctls                                                                     */
/* -------------------------------------------------------------------------- */

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, HI_ENC_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "Hisilicon VENC", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", HI_ENC_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt_out(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	const struct hi_v4l2_fmt *fmt = hi_v4l2_enum_fmt(f->index, HI_V4L2_RAW);

	if (!fmt)
		return -EINVAL;
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_enum_fmt_cap(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	const struct hi_v4l2_fmt *fmt = hi_v4l2_enum_fmt(f->index, HI_V4L2_ENC);

	if (!fmt)
		return -EINVAL;
	f->pixelformat = fmt->fourcc;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hi_enc_ctx *ctx = file2ctx(file);
	struct hi_enc_q_data *q = get_q_data(ctx, f->type);

	f->fmt.pix.width        = q->width;
	f->fmt.pix.height       = q->height;
	f->fmt.pix.field        = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat  = q->fmt->fourcc;
	f->fmt.pix.bytesperline = q->bytesperline;
	f->fmt.pix.sizeimage    = q->sizeimage;
	f->fmt.pix.colorspace   = ctx->colorspace;
	f->fmt.pix.ycbcr_enc    = ctx->ycbcr_enc;
	f->fmt.pix.quantization = ctx->quantization;
	f->fmt.pix.xfer_func    = ctx->xfer_func;
	return 0;
}

static int vidioc_try_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	const struct hi_v4l2_fmt *fmt;
	u32 bpl;

	fmt = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_RAW);
	if (!fmt) {
		fmt = hi_v4l2_find_fmt(V4L2_PIX_FMT_NV12, HI_V4L2_RAW);
		f->fmt.pix.pixelformat = fmt->fourcc;
	}
	f->fmt.pix.width  = clamp_t(u32, f->fmt.pix.width,  16, HI_ENC_MAX_W);
	f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 16, HI_ENC_MAX_H);
	f->fmt.pix.field  = V4L2_FIELD_NONE;
	f->fmt.pix.sizeimage = hi_v4l2_raw_sizeimage(f->fmt.pix.pixelformat,
						     f->fmt.pix.width,
						     f->fmt.pix.height, &bpl);
	f->fmt.pix.bytesperline = bpl;
	/* OUTPUT (raw) colorimetry is app-defined; default it when left unset */
	if (f->fmt.pix.colorspace == V4L2_COLORSPACE_DEFAULT) {
		f->fmt.pix.colorspace   = V4L2_COLORSPACE_REC709;
		f->fmt.pix.ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
		f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
		f->fmt.pix.xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	}
	return 0;
}

static int vidioc_try_fmt_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct hi_enc_ctx *ctx = file2ctx(file);
	const struct hi_v4l2_fmt *fmt;

	fmt = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_ENC);
	if (!fmt) {
		fmt = hi_v4l2_find_fmt(V4L2_PIX_FMT_H264, HI_V4L2_ENC);
		f->fmt.pix.pixelformat = fmt->fourcc;
	}
	f->fmt.pix.width  = clamp_t(u32, f->fmt.pix.width,  16, HI_ENC_MAX_W);
	f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 16, HI_ENC_MAX_H);
	f->fmt.pix.field  = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = 0;
	f->fmt.pix.sizeimage = coded_sizeimage(f->fmt.pix.width,
					       f->fmt.pix.height);
	/* CAPTURE colorimetry is fixed by the OUTPUT (raw) side */
	f->fmt.pix.colorspace   = ctx->colorspace;
	f->fmt.pix.ycbcr_enc    = ctx->ycbcr_enc;
	f->fmt.pix.quantization = ctx->quantization;
	f->fmt.pix.xfer_func    = ctx->xfer_func;
	return 0;
}

static int vidioc_s_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct hi_enc_ctx *ctx = file2ctx(file);
	struct hi_enc_q_data *q = &ctx->q_data[Q_SRC];
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;
	ret = vidioc_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	q->fmt          = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_RAW);
	q->width        = f->fmt.pix.width;
	q->height       = f->fmt.pix.height;
	q->sizeimage    = f->fmt.pix.sizeimage;
	q->bytesperline = f->fmt.pix.bytesperline;

	/* latch the app-provided colorimetry; CAPTURE inherits it */
	ctx->colorspace   = f->fmt.pix.colorspace;
	ctx->ycbcr_enc    = f->fmt.pix.ycbcr_enc;
	ctx->quantization = f->fmt.pix.quantization;
	ctx->xfer_func    = f->fmt.pix.xfer_func;

	/* keep CAPTURE (coded) geometry in sync */
	ctx->q_data[Q_DST].width     = q->width;
	ctx->q_data[Q_DST].height    = q->height;
	ctx->q_data[Q_DST].sizeimage = coded_sizeimage(q->width, q->height);
	ctx->params.width  = q->width;
	ctx->params.height = q->height;
	return 0;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct hi_enc_ctx *ctx = file2ctx(file);
	struct hi_enc_q_data *q = &ctx->q_data[Q_DST];
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;
	ret = vidioc_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	q->fmt       = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_ENC);
	q->width     = f->fmt.pix.width;
	q->height    = f->fmt.pix.height;
	q->sizeimage = f->fmt.pix.sizeimage;
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	return hi_v4l2_fill_framesize(fsize, HI_ENC_MAX_W, HI_ENC_MAX_H);
}

static int vidioc_try_encoder_cmd(struct file *file, void *priv,
				  struct v4l2_encoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_ENC_CMD_STOP && cmd->cmd != V4L2_ENC_CMD_START)
		return -EINVAL;
	return 0;
}

static int vidioc_encoder_cmd(struct file *file, void *priv,
			      struct v4l2_encoder_cmd *cmd)
{
	struct hi_enc_ctx *ctx = file2ctx(file);

	if (cmd->cmd != V4L2_ENC_CMD_STOP && cmd->cmd != V4L2_ENC_CMD_START)
		return -EINVAL;
	if (cmd->cmd == V4L2_ENC_CMD_START) {
		ctx->draining = false;
		return 0;
	}
	/* STOP: drain the encoder via capture-only jobs until no more coded
	 * output, then the last CAPTURE buffer carries V4L2_BUF_FLAG_LAST + EOS. */
	if (ctx->chan) {
		ctx->draining = true;
		ctx->drain_empty = 0;
		wake_up_interruptible(&ctx->cap_wq);	/* kthread drains the tail */
	}
	return 0;
}

static const struct v4l2_ioctl_ops hi_enc_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_cap,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_cap,

	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_out,
	.vidioc_g_fmt_vid_out		= vidioc_g_fmt,
	.vidioc_try_fmt_vid_out		= vidioc_try_fmt_out,
	.vidioc_s_fmt_vid_out		= vidioc_s_fmt_out,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_encoder_cmd		= vidioc_encoder_cmd,
	.vidioc_try_encoder_cmd		= vidioc_try_encoder_cmd,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* -------------------------------------------------------------------------- */
/* vb2 queue ops                                                              */
/* -------------------------------------------------------------------------- */

static int hi_enc_queue_setup(struct vb2_queue *vq, const void *parg,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], void *alloc_ctxs[])
{
	struct hi_enc_ctx *ctx = vb2_get_drv_priv(vq);
	struct hi_enc_q_data *q = get_q_data(ctx, vq->type);
	const struct v4l2_format *fmt = parg;
	u32 size = q->sizeimage;

	if (fmt) {
		/* VIDIOC_CREATE_BUFS: a plane too small for the format is invalid */
		if (fmt->fmt.pix.sizeimage < size)
			return -EINVAL;
		size = fmt->fmt.pix.sizeimage;
	}
	if (*nbuffers < 2)
		*nbuffers = 2;
	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = ctx->dev->alloc_ctx;
	return 0;
}

static int hi_enc_buf_prepare(struct vb2_buffer *vb)
{
	struct hi_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct hi_enc_q_data *q = get_q_data(ctx, vb->vb2_queue->type);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vb2_plane_size(vb, 0) < q->sizeimage) {
			dprintk(ctx->dev, "buf_prepare EINVAL plane_size=%lu sizeimage=%u\n",
				vb2_plane_size(vb, 0), q->sizeimage);
			return -EINVAL;
		}
		vb2_set_plane_payload(vb, 0, q->sizeimage);
	}
	return 0;
}

static void hi_enc_buf_queue(struct vb2_buffer *vb)
{
	struct hi_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int hi_enc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hi_enc_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	get_q_data(ctx, q->type)->sequence = 0;
	ctx->draining = false;

	if (!V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->cap_streaming = true;
		hi_enc_cap_thread_start(ctx);	/* starts iff chan also up */
		return 0;
	}
	if (ctx->chan) {
		hi_enc_cap_thread_start(ctx);
		return 0;
	}

	ctx->chan = hi_venc_hal_create(ctx->q_data[Q_DST].fmt, &ctx->params);
	if (IS_ERR(ctx->chan)) {
		ret = PTR_ERR(ctx->chan);
		ctx->chan = NULL;
		return ret;
	}
	ret = hi_venc_hal_start(ctx->chan);
	if (ret)
		return ret;
	hi_enc_cap_thread_start(ctx);
	return 0;
}

static void hi_enc_stop_streaming(struct vb2_queue *q)
{
	struct hi_enc_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;

	/* Tear down the capture thread BEFORE flushing so it can never race the
	 * flush or touch a buffer being returned. */
	ctx->cap_streaming = false;
	ctx->draining = false;
	wake_up_interruptible(&ctx->cap_wq);
	hi_enc_cap_thread_stop(ctx);

	/* Return any raw frames still parked in the VEDU in-flight FIFO. */
	for (;;) {
		vb = hi_enc_inflight_pop(ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}

	if (V4L2_TYPE_IS_OUTPUT(q->type) && ctx->chan) {
		hi_venc_hal_destroy(ctx->chan);
		ctx->chan = NULL;
	}
}

static struct vb2_ops hi_enc_qops = {
	.queue_setup	 = hi_enc_queue_setup,
	.buf_prepare	 = hi_enc_buf_prepare,
	.buf_queue	 = hi_enc_buf_queue,
	.start_streaming = hi_enc_start_streaming,
	.stop_streaming	 = hi_enc_stop_streaming,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct hi_enc_ctx *ctx = priv;
	int ret;

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes	= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	src_vq->ops		= &hi_enc_qops;
	src_vq->mem_ops		= &vb2_dma_contig_memops;
	src_vq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock		= &ctx->dev->dev_mutex;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes	= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops		= &hi_enc_qops;
	dst_vq->mem_ops		= &vb2_dma_contig_memops;
	dst_vq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	return vb2_queue_init(dst_vq);
}

/* -------------------------------------------------------------------------- */
/* file ops                                                                   */
/* -------------------------------------------------------------------------- */

static void init_q_defaults(struct hi_enc_ctx *ctx)
{
	struct hi_enc_q_data *src = &ctx->q_data[Q_SRC];
	struct hi_enc_q_data *dst = &ctx->q_data[Q_DST];

	src->fmt    = hi_v4l2_find_fmt(V4L2_PIX_FMT_NV12, HI_V4L2_RAW);
	src->width  = HI_ENC_DEF_W;
	src->height = HI_ENC_DEF_H;
	src->sizeimage = hi_v4l2_raw_sizeimage(src->fmt->fourcc, src->width,
					       src->height, &src->bytesperline);

	dst->fmt    = hi_v4l2_find_fmt(V4L2_PIX_FMT_H264, HI_V4L2_ENC);
	dst->width  = HI_ENC_DEF_W;
	dst->height = HI_ENC_DEF_H;
	dst->sizeimage = coded_sizeimage(dst->width, dst->height);

	ctx->params.width     = HI_ENC_DEF_W;
	ctx->params.height    = HI_ENC_DEF_H;
	ctx->params.framerate = 30;
	ctx->params.profile   = HI_UNF_H264_PROFILE_MAIN;

	ctx->colorspace   = V4L2_COLORSPACE_REC709;
	ctx->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	ctx->quantization = V4L2_QUANTIZATION_DEFAULT;
	ctx->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
}

static int hi_enc_open(struct file *file)
{
	struct hi_enc_dev *dev = video_drvdata(file);
	struct hi_enc_ctx *ctx;
	struct v4l2_ctrl_handler *hdl;
	int ret = 0;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->dev = dev;
	init_waitqueue_head(&ctx->cap_wq);
	spin_lock_init(&ctx->inflight_lock);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	init_q_defaults(ctx);

	hdl = &ctx->hdl;
	v4l2_ctrl_handler_init(hdl, 5);
	v4l2_ctrl_new_std(hdl, &hi_enc_ctrl_ops, V4L2_CID_MPEG_VIDEO_BITRATE,
			  16000, 60000000, 1000, 4000000);
	v4l2_ctrl_new_std(hdl, &hi_enc_ctrl_ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			  0, 300, 1, 30);
	v4l2_ctrl_new_std_menu(hdl, &hi_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, 0,
			       V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	/* H264 level — standard clients (gst/ffmpeg/tvheadend) query it during
	 * negotiation; without it G_CTRL returns -EINVAL and they bail. HW = 4.1. */
	v4l2_ctrl_new_std_menu(hdl, &hi_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_4_1, 0,
			       V4L2_MPEG_VIDEO_H264_LEVEL_4_1);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto unlock;
	}
	ctx->fh.ctrl_handler = hdl;
	v4l2_ctrl_handler_setup(hdl);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_ctrl_handler_free(hdl);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto unlock;
	}
	/* Let the EOS drain run capture-only jobs (empty OUTPUT queue). */
	v4l2_m2m_set_src_buffered(ctx->fh.m2m_ctx, true);

	v4l2_fh_add(&ctx->fh);
	atomic_inc(&dev->num_inst);
unlock:
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int hi_enc_release(struct file *file)
{
	struct hi_enc_dev *dev = video_drvdata(file);
	struct hi_enc_ctx *ctx = file2ctx(file);

	mutex_lock(&dev->dev_mutex);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (ctx->chan)
		hi_venc_hal_destroy(ctx->chan);
	mutex_unlock(&dev->dev_mutex);

	kfree(ctx);
	atomic_dec(&dev->num_inst);
	return 0;
}

static const struct v4l2_file_operations hi_enc_fops = {
	.owner		= THIS_MODULE,
	.open		= hi_enc_open,
	.release	= hi_enc_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct video_device hi_enc_videodev = {
	.name		= HI_ENC_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &hi_enc_fops,
	.ioctl_ops	= &hi_enc_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
};

/* -------------------------------------------------------------------------- */
/* platform driver                                                            */
/* -------------------------------------------------------------------------- */

/*
 * VENC encode-capability probe (replaces the old hardcoded "H264 only"): a real
 * try-create per encode candidate. The VEDU rejects unsupported codecs at create
 * (e.g. HEVC -> "NOT support Type 36") and JPEG needs the JPGE block present
 * (pJpgeFunc); only codecs whose create succeeds are advertised. 640x480/Main is
 * a shape that fails only for genuine codec-unsupport, not for bad parameters.
 */
static bool hi_enc_can_encode(const struct hi_v4l2_fmt *fmt)
{
	struct hi_venc_params p = {
		.width = 640, .height = 480, .framerate = 30,
		.profile = HI_UNF_H264_PROFILE_MAIN,
	};
	struct hi_venc_chan *c = hi_venc_hal_create(fmt, &p);

	if (IS_ERR(c))
		return false;
	hi_venc_hal_destroy(c);
	return true;
}

static int hi_enc_probe(struct platform_device *pdev)
{
	struct hi_enc_dev *dev;
	struct video_device *vfd;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->dev_mutex);
	atomic_set(&dev->num_inst, 0);

	ret = hi_venc_hal_global_init();
	if (ret)
		return ret;

	/* Advertise only the codecs the VENC actually encodes (real probe). */
	pr_info("hi-v4l2: VENC accepts %d encode format(s)\n",
		hi_v4l2_probe_enc_caps(hi_enc_can_encode));

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_hal;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_hal;

	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		ret = PTR_ERR(dev->alloc_ctx);
		goto err_v4l2;
	}

	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		ret = PTR_ERR(dev->m2m_dev);
		goto err_ctx;
	}

	dev->vfd = hi_enc_videodev;
	vfd = &dev->vfd;
	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret)
		goto err_m2m;

	video_set_drvdata(vfd, dev);
	platform_set_drvdata(pdev, dev);
	v4l2_info(&dev->v4l2_dev, "encoder registered as /dev/video%d\n",
		  vfd->num);
	return 0;

err_m2m:
	v4l2_m2m_release(dev->m2m_dev);
err_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_hal:
	hi_venc_hal_global_exit();
	return ret;
}

static int hi_enc_remove(struct platform_device *pdev)
{
	struct hi_enc_dev *dev = platform_get_drvdata(pdev);

	video_unregister_device(&dev->vfd);
	v4l2_m2m_release(dev->m2m_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_device_unregister(&dev->v4l2_dev);
	hi_venc_hal_global_exit();
	return 0;
}

static void hi_enc_pdev_release(struct device *dev) {}

static struct platform_device hi_enc_pdev = {
	.name		= HI_ENC_NAME,
	.dev.release	= hi_enc_pdev_release,
};

static struct platform_driver hi_enc_pdrv = {
	.probe		= hi_enc_probe,
	.remove		= hi_enc_remove,
	.driver		= {
		.name	= HI_ENC_NAME,
	},
};

int hi_enc_register(void)
{
	int ret = platform_device_register(&hi_enc_pdev);

	if (ret)
		return ret;
	ret = platform_driver_register(&hi_enc_pdrv);
	if (ret)
		platform_device_unregister(&hi_enc_pdev);
	return ret;
}

void hi_enc_unregister(void)
{
	platform_driver_unregister(&hi_enc_pdrv);
	platform_device_unregister(&hi_enc_pdev);
}
