/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Thin HAL over the in-kernel, EXPORT_SYMBOL'd HI_DRV_VDEC_* API
 * (source/msp/drv/vdec/vdec_v2.0). This is the same decode layer the
 * userspace AVPLAY drives, so it is the proven path (vs. the unstable OMX).
 *
 * All HI_DRV_VDEC_* entry points used here are exported:
 *   drv_vdec_intf_k.c:17571+ (Open/Close/AllocChan/SetChanAttr/ChanStart/Stop/
 *   GetEsBuf/PutEsBuf/SetEosFlag/RecvFrmBuf/CreateStrmBuf/CreatePort/.../GetCap)
 *
 * Points whose exact semantics must be confirmed on real Hi3798Cv200 hardware
 * are tagged HW-VALIDATE in the .c file.
 */
#ifndef __HI_VDEC_HAL_H__
#define __HI_VDEC_HAL_H__

#include <linux/types.h>
#include "hi_v4l2_fmt.h"

struct hi_vdec_chan;

/* One decoded picture, still owned by the decoder until _release_frame(). */
struct hi_vdec_frame {
	HI_DRV_VIDEO_FRAME_S	raw;        /* native handle, kept for release */
	u32			width;
	u32			height;
	u32			stride_y;
	u32			stride_c;
	u32			phys_y;     /* MMZ physical addr (DMABUF source)   */
	u32			phys_c;
	void			*virt_y;    /* kernel mapping, may be NULL         */
	void			*virt_c;
	u64			pts;
	bool			last;       /* set on the EOS sentinel frame       */
	void			*map_base;  /* win path: ioremap'd base, NULL if none */
	u32			map_size;   /* win path: span to iounmap on release   */
};

/* Global module bring-up (refcounted): HI_DRV_VDEC_Open / _Close. */
int  hi_vdec_hal_global_init(void);
void hi_vdec_hal_global_exit(void);

/* Per-instance channel lifecycle. */
struct hi_vdec_chan *hi_vdec_hal_create(const struct hi_v4l2_fmt *coded,
					u32 width, u32 height,
					u32 es_buf_size);
void hi_vdec_hal_destroy(struct hi_vdec_chan *c);
int  hi_vdec_hal_start(struct hi_vdec_chan *c);
int  hi_vdec_hal_stop(struct hi_vdec_chan *c);

/* Feed one coded access unit; @eof marks an end-of-frame boundary. */
int  hi_vdec_hal_feed_es(struct hi_vdec_chan *c, const void *data, u32 len,
			 u64 pts, bool eof);
int  hi_vdec_hal_set_eos(struct hi_vdec_chan *c);

/* Non-blocking: 0 = frame returned, -EAGAIN = none ready yet. */
int  hi_vdec_hal_recv_frame(struct hi_vdec_chan *c, struct hi_vdec_frame *f);
void hi_vdec_hal_release_frame(struct hi_vdec_chan *c, struct hi_vdec_frame *f);

/* Enable de-tile prerequisites globally on the vendor VDEC (map_frm + maskcmp).
 * Call before ChanStart when the detile path is used; the SW un-tile then runs
 * in copy_frame_to_buf (untile_plane). */
void hi_vdec_hal_enable_detile(void);

/*
 * Zero-copy Fase C (KI-003): VPSS user-managed buffers. Commits caller-owned
 * (CMA) frame buffers to the VPSS, which decodes straight into them (no
 * VFMW->CAPTURE memcpy), then receives/releases by handle via the
 * HI_DRV_VDEC_UserBuf_* wrappers. Proven on HW 2026-06-12. Used by the
 * production decoder when the `zc` module param is set, and by the isolated
 * debugfs probe (hi_v4l2_zc_test.c, gated by HI_V4L2_ZC_SELFTEST).
 *
 * zc_create builds a real VPSS HD/MASTER port (not the VFMW-direct STR bypass)
 * AND sets USER_ALLOC mode before CreatePort (mandatory order, see .c). Commit
 * the frame buffers, then hi_vdec_hal_start(); recv returns the frame in one of
 * the committed buffers (match by phys); release returns it to the VPSS pool.
 */
struct hi_vdec_chan *hi_vdec_hal_zc_create(const struct hi_v4l2_fmt *coded,
					   u32 width, u32 height, u32 es_buf_size);
int  hi_vdec_hal_zc_commit(struct hi_vdec_chan *c, u32 *phys, u64 *virt,
			   u32 num, u32 size, u32 stride);
int  hi_vdec_hal_zc_recv(struct hi_vdec_chan *c, struct hi_vdec_frame *f);
void hi_vdec_hal_zc_release(struct hi_vdec_chan *c, struct hi_vdec_frame *f);

/*
 * VO virtual-window path (KI-003 / T0!): the proven way the SDK turns the
 * decoder's native (VCMP-compressed + 2D-tiled) frame into LINEAR NV12. The
 * VFMW-direct path (RecvFrmBuf) reads that native frame raw -> garbage; only
 * the VPSS linearizes it, and the VPSS only runs when a VO window consumes the
 * port (it is the window/VO that drives the pipeline). This mirrors libhicodec
 * (HI_UNF_VO_CreateWindow[virtual] + AttachWindow + AcquireFrame): we build a
 * VPSS HD/MASTER port (VPSS-managed buffers) and attach a virtual VO window via
 * HI_DRV_WIN_*; recv = WIN_AcquireFrame (linear NV12), release = WIN_ReleaseFrame.
 * Frames go through the normal hi_vdec_hal_recv_frame/_release_frame, which
 * dispatch on the window handle, so the existing memcpy drain works unchanged.
 */
struct hi_vdec_chan *hi_vdec_hal_win_create(const struct hi_v4l2_fmt *coded,
					    u32 width, u32 height, u32 es_buf_size);

/*
 * Variant B (vp): VPSS-managed decode without a VO window. VPSS allocates and
 * processes its own linear NV12; frames pulled with RecvVpssFrmBuf + memcpy.
 * recv/release route through hi_vdec_hal_recv_frame/_release_frame (dispatch on
 * the chan's vpss_recv flag). See the .c for the rationale vs. the win path.
 */
struct hi_vdec_chan *hi_vdec_hal_vp_create(const struct hi_v4l2_fmt *coded,
					   u32 width, u32 height, u32 es_buf_size);

#endif /* __HI_VDEC_HAL_H__ */
