/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Thin HAL over the in-kernel, EXPORT_SYMBOL'd HI_DRV_VENC_* API
 * (source/msp/drv/venc/venc_v2.0). Same proven encode layer AVPLAY/transcode use.
 *
 * Exported entry points used here (kernel Module.symvers):
 *   Init/DeInit/GetDefaultAttr/Create/Destroy/Start/Stop/QueueFrame/DequeueFrame/
 *   AcquireStream/ReleaseStream/RequestIFrame/SetAttr/GetAttr
 *
 * Points needing on-target confirmation are tagged HW-VALIDATE in the .c file.
 */
#ifndef __HI_VENC_HAL_H__
#define __HI_VENC_HAL_H__

#include <linux/types.h>
#include "hi_drv_venc.h"   /* HI_UNF_VENC_CHN_ATTR_S, VENC_STREAM_S, frame info */
#include "hi_v4l2_fmt.h"

struct hi_venc_chan;

/* Encoder open parameters (V4L2 controls feed these). */
struct hi_venc_params {
	u32	width;
	u32	height;
	u32	bitrate;     /* bps; 0 => driver default */
	u32	framerate;   /* fps */
	u32	gop;         /* 0 => driver default */
	u32	profile;     /* HI_UNF_H264_PROFILE_* */
};

/* One encoded chunk, owned by the encoder until _release_stream(). */
struct hi_venc_stream {
	void	*data;       /* kernel VA of the bitstream */
	u32	len;
	u64	pts;         /* ms */
	bool	frame_end;
	VENC_STREAM_S raw;   /* native handle, kept for release */
};

int  hi_venc_hal_global_init(void);
void hi_venc_hal_global_exit(void);

struct hi_venc_chan *hi_venc_hal_create(const struct hi_v4l2_fmt *coded,
					const struct hi_venc_params *p);
void hi_venc_hal_destroy(struct hi_venc_chan *c);
int  hi_venc_hal_start(struct hi_venc_chan *c);
int  hi_venc_hal_stop(struct hi_venc_chan *c);
int  hi_venc_hal_request_keyframe(struct hi_venc_chan *c);

/* Feed one raw frame by physical address (NV12 if !nv21, else NV21). */
int  hi_venc_hal_queue_frame(struct hi_venc_chan *c, u32 phys_y, u32 ystride,
			     u32 width, u32 height, u64 pts_ms, bool nv21);
/* Reclaim a consumed input frame (non-blocking, best-effort). */
void hi_venc_hal_dequeue_frame(struct hi_venc_chan *c);

/* Non-blocking: 0 = stream returned, -EAGAIN = none ready. */
int  hi_venc_hal_acquire_stream(struct hi_venc_chan *c, struct hi_venc_stream *s);
void hi_venc_hal_release_stream(struct hi_venc_chan *c, struct hi_venc_stream *s);

#endif /* __HI_VENC_HAL_H__ */
