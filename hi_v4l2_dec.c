// SPDX-License-Identifier: GPL-2.0
/*
 * Hisilicon V4L2 memory-to-memory stateful video decoder.
 *
 * OUTPUT  queue: coded bitstream (H264/HEVC/MPEG2/MPEG4/VP8/VP9/VC1/MJPEG...)
 * CAPTURE queue: decoded frames (NV12/NV21).
 *                PHASE1 uses vb2-vmalloc for bring-up; switching to vb2-dma-contig
 *                for DMABUF zero-copy is tracked in docs/known-issues.md (KI-003).
 *
 * Backend: HI_DRV_VDEC_* (vdec_v2.0), via hi_vdec_hal.c -- the same proven decode
 * layer AVPLAY uses (the Hisilicon OMX path is unstable and intentionally avoided).
 *
 * Phase 1 scope: a synchronous 1-AU-in / 1-frame-out transaction per device_run().
 * Decoupled feed/capture (true bitstream buffering, B-frame reorder depth, dynamic
 * resolution via V4L2_EVENT_SOURCE_CHANGE) is the next phase; the simplification is
 * marked PHASE1 below.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/platform_device.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>

#include "hi_v4l2_fmt.h"
#include "hi_vdec_hal.h"
#include "hi_v4l2_drv.h"

#define HI_DEC_NAME		"hi-vdec"
#define HI_DEC_ES_BUF_SIZE	(4 * 1024 * 1024)
#define HI_DEC_DEF_W		1280
#define HI_DEC_DEF_H		720
#define HI_DEC_MAX_W		4096
#define HI_DEC_MAX_H		4096
#define HI_DEC_RECV_RETRIES	250	/* PHASE1: ~500ms worst case (250 x 2ms poll) */
#define HI_DEC_DRAIN_EMPTY_MAX	20	/* empty drain polls (20 x 2ms = ~40ms quiet) before EOS */
/* Capture kthread (T5): drains frames decoupled from feed. */
#define HI_DEC_CAP_POLL_US_MIN	1000	/* inter-poll sleep while frames decode */
#define HI_DEC_CAP_POLL_US_MAX	2000
/*
 * Consecutive empty polls (since the last delivered frame) before the thread
 * assumes the remaining in-flight count is unreachable (e.g. VPSS tail frames,
 * KI-002, or a non-displayed AU that left inflight drifted) and parks. Must be
 * WELL above the worst real inter-frame gap (startup pipeline-fill ~170ms,
 * B-frame reorder) so it never abandons a backlog that is merely decoding.
 * ~600 x ~1.5ms ~= 900ms.
 */
#define HI_DEC_CAP_IDLE_MAX	600
/*
 * Back-pressure cap: how many fed-but-not-yet-output frames may pile up before
 * feeding (job_ready) is held off; the capture thread re-arms it as it drains.
 * ADAPTIVE (codec-agnostic):
 *  - BASE (8): default. Keeps the backlog small so a feed-then-stop consumer with
 *    a weak single-batch EOS drain (ffmpeg 4.1) still retrieves it, and paces such
 *    consumers to the decode rate.
 *  - REORDER (32): used only while the decoder is REORDER-FILLING — consuming
 *    input without producing output for longer than normal decode latency (deep
 *    B-frame reorder like HEVC's B-pyramid, or startup). A fixed small cap would
 *    DEADLOCK the feed against the reorder fill (decoder idles waiting for input
 *    the cap won't let us send). The capture thread detects this (HI_DEC_STARVE_MAX
 *    empty polls since the last delivered frame) and sets ctx->reorder_fill; it is
 *    cleared on the next output, so the BASE cap resumes for pacing. No per-codec
 *    value -> robust for any codec / reorder depth.
 * HI_DEC_STARVE_MAX must exceed the normal inter-output gap (the kthread resets the
 * idle counter on every delivery) so steady real-time flow never trips it.
 */
#define HI_DEC_MAX_INFLIGHT		8
#define HI_DEC_MAX_INFLIGHT_REORDER	32
#define HI_DEC_STARVE_MAX		50	/* ~75ms of no output @ ~1.5ms/poll */
/*
 * Watchdog: if the decoder produces NO output for this long while input is still
 * in flight (even after the cap was lifted for reorder), it is wedged — typically
 * malformed/unsupported ES it can't make progress on. Rather than hang the
 * consumer's dqbuf/poll forever, signal EOS so it finishes/errors gracefully.
 * Far above any real decode latency / deep reorder, so it never trips on a
 * merely-slow stream (a working decoder refreshes the timer on every frame).
 */
#define HI_DEC_WATCHDOG_MS		3000

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

/*
 * Zero-copy CAPTURE via the VPSS user-managed path (KI-003 Fase C). Off by
 * default: the proven VFMW-direct + memcpy path stays the default. When set,
 * the CAPTURE vb2 (dma-contig/CMA) buffers are committed to the VPSS and the
 * decoder writes NV12 straight into them (no copy_frame_to_buf memcpy).
 */
static bool zc;
module_param(zc, bool, 0644);
MODULE_PARM_DESC(zc, "zero-copy CAPTURE via VPSS user-managed buffers (KI-003)");

/*
 * Decode through a virtual VO window (T0!): the VFMW-direct path delivers the
 * decoder's NATIVE frame (VCMP-compressed + 2D-tiled) which reads as garbage;
 * only the VPSS linearizes it, and the VPSS only runs when a VO window consumes
 * the port. With win=1 we build a VPSS master port + virtual VO window and pull
 * LINEAR NV12 via WIN_AcquireFrame (then the usual memcpy drain). This is the
 * libhicodec recipe in-kernel. Off by default; takes precedence checked after
 * zc, so zc=1 still wins if both are set.
 */
static bool win;
module_param(win, bool, 0644);
MODULE_PARM_DESC(win, "decode via virtual VO window (VPSS-linearized NV12, T0!)");

/*
 * Variant B (vp): VPSS-managed decode WITHOUT a VO window — the VPSS allocates
 * and processes its own linear NV12, pulled with RecvVpssFrmBuf + memcpy. Tests
 * whether a pull consumer alone drives VPSS linearization (no window push, no
 * master-port free-run). Off by default; checked after zc and win.
 */
static bool vp;
module_param(vp, bool, 0644);
MODULE_PARM_DESC(vp, "decode via VPSS-managed RecvVpssFrmBuf, no window (T0! variant B)");

/*
 * De-tile the decoder's native (uncompressed, tiled) frame to LINEAR NV12 in
 * software, in copy_frame_to_buf via untile_plane(). The default VFMW-direct
 * path (RecvFrmBuf) yields a 64x16-tiled NV21_TILE frame; untile_plane()
 * rewrites it to linear using the exact vendor algorithm + row-map tables (see
 * untile_plane). The driver enables the prerequisites itself when detile is set
 * (hi_vdec_hal_enable_detile: map_frm + maskcmp via exported vendor globals).
 * Validated pixel-accurate vs software ground-truth on natural video (BBB);
 * see docs/vpss-detile-investigation.md §37. ~2 fps @1080p (uncached source).
 *
 * Default ON: a generic V4L2 consumer (GStreamer/FFmpeg/Kodi) requires LINEAR
 * NV12. With de-tile off the CAPTURE buffer carries the raw 64x16-tiled frame
 * (renders green/garbage), which is useless to any standard client. Set
 * detile=0 only to expose the native tiled frame (no current consumer).
 */
static bool detile = true;
module_param(detile, bool, 0644);
MODULE_PARM_DESC(detile, "de-tile native frame to linear NV12 in software (default on; 0 = raw tiled)");

#define dprintk(d, fmt, arg...) \
	v4l2_dbg(1, debug, &(d)->v4l2_dev, "%s: " fmt, __func__, ## arg)

enum { Q_SRC = 0, Q_DST = 1 };

struct hi_dec_q_data {
	u32				width;
	u32				height;
	u32				sizeimage;
	u32				bytesperline;
	u32				sequence;
	const struct hi_v4l2_fmt	*fmt;
};

struct hi_dec_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct platform_device	*pdev;
	struct mutex		dev_mutex;
	struct v4l2_m2m_dev	*m2m_dev;
	void			*alloc_ctx;	/* dma-contig ctx for CAPTURE */
	atomic_t		num_inst;
};

struct hi_dec_ctx {
	struct v4l2_fh		fh;
	struct hi_dec_dev	*dev;
	struct hi_vdec_chan	*chan;
	struct hi_dec_q_data	q_data[2];
	bool			aborting;
	bool			draining;	/* EOS drain in progress (DEC_CMD_STOP) */
	int			drain_empty;	/* consecutive empty polls while draining */
	/*
	 * Capture kthread (T5): decouples frame capture from feed. device_run only
	 * feeds the decoder (non-blocking) and wakes cap_thread; the thread pulls
	 * decoded frames (RecvFrmBuf), de-tiles OUTSIDE any lock, and buf_done()s
	 * CAPTURE buffers — independent of the OUTPUT cadence. This gives both
	 * GStreamer (real-time) and ffmpeg (no DEC_CMD_STOP) correct draining.
	 */
	struct task_struct	*cap_thread;
	wait_queue_head_t	cap_wq;
	atomic_t		inflight;	/* fed AUs not yet output as frames */
	bool			cap_streaming;	/* CAPTURE queue is streaming */
	bool			reorder_fill;	/* decoder is reorder-filling (consuming
						 * input w/o output, e.g. HEVC B-pyramid
						 * or startup) -> lift the feed cap so it
						 * isn't deadlocked; cleared on next output */
	unsigned long		last_out_jiffies; /* last frame delivered — watchdog */
	/* zero-copy CAPTURE (KI-003): VPSS writes into our vb2 buffers, no memcpy */
	bool			zc;		/* this instance uses zero-copy */
	bool			zc_started;	/* CAPTURE buffers committed + ChanStart */
	u32			zc_phys[VIDEO_MAX_FRAME];	/* per vb2 index */
	bool			zc_out[VIDEO_MAX_FRAME];	/* handed to app; release on requeue */
	struct hi_vdec_frame	zc_frame[VIDEO_MAX_FRAME];	/* frame to release */
	/* colorimetry: set on OUTPUT, propagated to CAPTURE (V4L2 codec API) */
	u32			colorspace;
	u32			ycbcr_enc;
	u32			quantization;
	u32			xfer_func;
};

static inline struct hi_dec_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct hi_dec_ctx, fh);
}

static struct hi_dec_q_data *get_q_data(struct hi_dec_ctx *ctx,
					enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->q_data[Q_SRC]
					 : &ctx->q_data[Q_DST];
}

/* -------------------------------------------------------------------------- */
/* mem2mem ops                                                                */
/* -------------------------------------------------------------------------- */

/*
 * De-tile one byte plane (NV21_TILE) to LINEAR — the EXACT vendor algorithm,
 * transcribed from the OMX vdec C source (omxvdec_v2.0/processor_vpss.c
 * processor_save_yuv_NV21_Tile) + the vendor row-map tables
 * (vfmw_row_map_table.h, CHIP_TYPE_hi3798cv200). 8-bit path.
 *
 * The plane is 64-byte tiles, TH rows tall (TH=16 luma, 8 chroma), tiles
 * left-to-right, TH-line bands top-to-bottom (band stride = stride_y*TH). The
 * row order WITHIN a tile is NOT a simple field-interleave — it is a lookup
 * `tbl[tileY][tileX][srcIdx]` where tileY=(band)%2, tileX=(tilecol)%4,
 * srcIdx=row%TH. (My earlier fixed field-interleave was only an approximation —
 * right for some tileX, wrong for others; it survived color bars but scrambled
 * real video / chroma.)  src = base + stride_y*TH*(y/TH) + dstIdx*64 +
 * (x/64)*64*TH.  Chroma de-tiles to interleaved NV21 (V,U); caller swaps for NV12.
 */
static const int g_RowMapTable_Y[2][4][16] = {
	{ {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15},
	  {12, 0, 13, 1, 14, 2, 15, 3, 8, 4, 9, 5, 10, 6, 11, 7},
	  {4, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11},
	  {8, 4, 9, 5, 10, 6, 11, 7, 12, 0, 13, 1, 14, 2, 15, 3} },
	{ {8, 0, 9, 1, 10, 2, 11, 3, 12, 4, 13, 5, 14, 6, 15, 7},
	  {0, 12, 1, 13, 2, 14, 3, 15, 4, 8, 5, 9, 6, 10, 7, 11},
	  {12, 4, 13, 5, 14, 6, 15, 7, 8, 0, 9, 1, 10, 2, 11, 3},
	  {4, 8, 5, 9, 6, 10, 7, 11, 0, 12, 1, 13, 2, 14, 3, 15} },
};
static const int g_RowMapTable_UV[2][4][8] = {
	{ {0, 4, 1, 5, 2, 6, 3, 7}, {4, 0, 5, 1, 6, 2, 7, 3},
	  {4, 0, 5, 1, 6, 2, 7, 3}, {0, 4, 1, 5, 2, 6, 3, 7} },
	{ {4, 0, 5, 1, 6, 2, 7, 3}, {0, 4, 1, 5, 2, 6, 3, 7},
	  {0, 4, 1, 5, 2, 6, 3, 7}, {4, 0, 5, 1, 6, 2, 7, 3} },
};

static void untile_plane(u8 *dst, const u8 *src, u32 w, u32 h,
			 u32 dst_stride, u32 stride_y, u32 tile_h,
			 const int *tbl, bool swap_pairs)
{
	u32 band_stride = stride_y * tile_h;
	u32 tile_step   = 64 * tile_h;          /* one tile, contiguous */
	u32 bands = (h + tile_h - 1) / tile_h;
	u32 band, x, r;
	u8 stage[64 * 16];                      /* one tile; tile_h <= 16 */

	/*
	 * The source mapping (map_frm) is UNCACHED. Reading the de-tiled output
	 * directly from it does scattered 64-byte reads (each its own cache miss,
	 * no burst) -> ~150 ms/frame @1080p. Each tile is 64*tile_h CONTIGUOUS
	 * bytes though, so we copy the whole tile sequentially (uncached burst is
	 * fast) into a cached scratch, then scatter its rows from there. Same
	 * pixels, ~8x faster. No cached alias of the uncached mapping (that would
	 * be unsafe on ARMv7) — we only change the read ORDER.
	 */
	for (band = 0; band < bands; band++) {
		u32 tile_y = band & 1;
		const u8 *bb = src + band_stride * band;
		u32 base_y = band * tile_h;

		for (x = 0; x < w; x += 64) {
			u32 tile_x = (x / 64) & 3;
			u32 n = (x + 64 <= w) ? 64 : (w - x);

			memcpy(stage, bb + (x / 64) * tile_step, tile_step);

			for (r = 0; r < tile_h; r++) {
				u32 y = base_y + r;
				u32 dst_idx;
				u8 *ss;

				if (y >= h)
					break;
				dst_idx = tbl[(tile_y * 4 + tile_x) * tile_h + r];
				ss = stage + dst_idx * 64;
				/*
				 * NV21->NV12 chroma swap, done HERE on the cached
				 * scratch. Doing it as a separate in-place pass over
				 * the vb2 buffer cost ~150 ms/frame (byte r-m-w on the
				 * write-combine CAPTURE buffer); here it is ~free.
				 */
				if (swap_pairs) {
					u32 k;

					for (k = 0; k + 1 < n; k += 2)
						swap(ss[k], ss[k + 1]);
				}
				memcpy(dst + y * dst_stride + x, ss, n);
			}
		}
	}
}

static void copy_frame_to_buf(struct hi_dec_ctx *ctx, struct hi_vdec_frame *fr,
			      struct vb2_v4l2_buffer *dst)
{
	struct hi_dec_q_data *q = &ctx->q_data[Q_DST];
	u8 *out = vb2_plane_vaddr(&dst->vb2_buf, 0);
	u8 *sy = fr->virt_y, *sc = fr->virt_c;
	u32 bpl = q->bytesperline;
	u32 w = min(fr->width, q->width);
	u32 h = min(fr->height, q->height);
	u32 y;

	if (!out || !sy || !sc) {
		/* PHASE1: pixel data lives in MMZ; a NULL kernel mapping means
		 * we must wrap MMZ as the dma-buf instead of copying (next phase).
		 */
		vb2_set_plane_payload(&dst->vb2_buf, 0, q->sizeimage);
		return;
	}

	/*
	 * SW de-tile: the source is the decoder's native 64x16-tiled NV21_TILE
	 * frame (uncompressed: maskcmp on). untile_plane() rewrites it to LINEAR.
	 * Y plane: w x h; chroma plane (NV21/NV12 CbCr interleaved): w x h/2, half
	 * the lines, same byte-tiling. Writes Y at `out`, chroma at out+bpl*height.
	 * Validated pixel-accurate vs software ground-truth (BBB, §37).
	 */
	if (detile) {
		/*
		 * Direct un-tile off the (uncached map_frm) source. Correct + stable.
		 * NOTE: slow (~2 fps 1080p) — the source mapping is uncached and the
		 * un-tile does scattered reads. Real-time would need a cached source
		 * mapping or a PERSISTENT per-ctx cached scratch + dma_sync (a per-frame
		 * vmalloc scratch was tried and caused instability). Future perf work.
		 *
		 * Both planes use the LUMA stride for the band stride (vendor:
		 * luma band = stride_y*16, chroma band = stride_y*16/2 = stride_y*8).
		 */
		/*
		 * The decoder's native chroma is NV21 (Cr,Cb interleaved). For an
		 * NV12 (Cb,Cr) request, swap each chroma byte-pair — folded into the
		 * chroma untile (on the cached scratch), not a separate vb2 pass.
		 */
		bool swap_c = (q->fmt->fourcc == V4L2_PIX_FMT_NV12);

		untile_plane(out, sy, w, h, bpl, fr->stride_y, 16,
			     &g_RowMapTable_Y[0][0][0], false);
		untile_plane(out + bpl * q->height, sc, w, h / 2,
			     bpl, fr->stride_y, 8, &g_RowMapTable_UV[0][0][0],
			     swap_c);
		vb2_set_plane_payload(&dst->vb2_buf, 0, q->sizeimage);
		return;
	}

	for (y = 0; y < h; y++)                       /* luma */
		memcpy(out + y * bpl, sy + y * fr->stride_y, w);

	out += bpl * q->height;                       /* chroma (NV12: CbCr) */
	for (y = 0; y < h / 2; y++)
		memcpy(out + y * bpl, sc + y * fr->stride_c, w);

	vb2_set_plane_payload(&dst->vb2_buf, 0, q->sizeimage);
}

/*
 * T5 — decoupled feed/capture model (KI-002/KI-015).
 *
 * The decoder is pipelined (~6 frames latency, B-frame reorder), so input and
 * output do NOT line up 1:1. Two rules keep this safe on the 4.4 v4l2-mem2mem:
 *
 *  - NEVER emit an empty CAPTURE buffer. We only buf_done() a dst that carries
 *    a real frame (GStreamer treats a 0-byte CAPTURE buffer as corrupt — the
 *    root cause of KI-015). Output uses the frame's own PTS, not the src's.
 *  - ONE device_run() drains the whole ready batch (feed every queued src,
 *    draining ready frames between feeds) and then calls job_finish() once.
 *    Because job_finish()->try_schedule()->try_run()->device_run() recurses on
 *    the same stack, consuming the batch in a single call keeps job_ready() at
 *    0 afterwards and avoids the deep recursion that overflowed the 8 KB kernel
 *    stack in the earlier per-buffer decoupled attempt (KI-015 crash).
 */

/* Output one decoded frame into a CAPTURE buffer (real payload, never empty). */
static void dec_output_frame(struct hi_dec_ctx *ctx, struct hi_vdec_frame *fr,
			     struct vb2_v4l2_buffer *dst)
{
	u64 pts = fr->pts;                               /* frame's own PTS (us) */
	u32 usec = do_div(pts, 1000000);                 /* 32-bit ARM: no u64 / */

	copy_frame_to_buf(ctx, fr, dst);
	dst->timestamp.tv_sec  = pts;
	dst->timestamp.tv_usec = usec;
	dst->sequence = ctx->q_data[Q_DST].sequence++;
	dst->field = V4L2_FIELD_NONE;
	dst->flags &= ~V4L2_BUF_FLAG_LAST;
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
}

/* Emit the empty LAST buffer + the EOS event and end the drain. */
static void dec_emit_eos(struct hi_dec_ctx *ctx, struct vb2_v4l2_buffer *dst)
{
	pr_debug("HI-DBG: EMIT_EOS (LAST) seq=%u draining=%d\n",
		 ctx->q_data[Q_DST].sequence, ctx->draining);
	vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	dst->sequence = ctx->q_data[Q_DST].sequence++;
	dst->field = V4L2_FIELD_NONE;
	dst->flags |= V4L2_BUF_FLAG_LAST;
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	ctx->draining = false;
	if (ctx->fh.m2m_ctx) {
		static const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };
		v4l2_event_queue_fh(&ctx->fh, &ev);
	}
}

/*
 * Drain all currently-ready frames into available CAPTURE buffers. Non-blocking
 * (recv_frame returns != 0 when the DPB has nothing ready). Returns the number
 * of real frames output; sets *eos when the VFMW end-of-stream sentinel hits.
 * recv_frame is only called when a dst is free, so a ready frame is never lost.
 */
static int dec_drain_ready(struct hi_dec_ctx *ctx, bool *eos)
{
	struct vb2_v4l2_buffer *dst;
	struct hi_vdec_frame fr;
	int n = 0;

	while ((dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx)) != NULL) {
		if (hi_vdec_hal_recv_frame(ctx->chan, &fr) != 0)
			break;                      /* DPB empty */
		if (fr.last) {                      /* EOS sentinel: owns no buffer */
			hi_vdec_hal_release_frame(ctx->chan, &fr);
			dec_emit_eos(ctx, dst);
			*eos = true;
			break;
		}
		dec_output_frame(ctx, &fr, dst);
		hi_vdec_hal_release_frame(ctx->chan, &fr);
		n++;
	}
	return n;
}

/* -------------------------------------------------------------------------- */
/* zero-copy CAPTURE (KI-003): VPSS writes NV12 into our vb2 buffers           */
/* -------------------------------------------------------------------------- */

/* Commit all allocated CAPTURE (dma-contig/CMA) buffers to the VPSS pool. */
static int zc_commit_buffers(struct hi_dec_ctx *ctx)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE);
	struct hi_dec_q_data *q = &ctx->q_data[Q_DST];
	u32 phys[VIDEO_MAX_FRAME];
	u64 virt[VIDEO_MAX_FRAME];
	unsigned int i, n;

	if (!vq || vq->num_buffers == 0 || vq->num_buffers > VIDEO_MAX_FRAME)
		return -EINVAL;
	n = vq->num_buffers;
	for (i = 0; i < n; i++) {
		struct vb2_buffer *vb = vq->bufs[i];

		phys[i] = (u32)vb2_dma_contig_plane_dma_addr(vb, 0);
		virt[i] = (u64)(unsigned long)vb2_plane_vaddr(vb, 0);
		ctx->zc_phys[vb->index] = phys[i];
	}
	return hi_vdec_hal_zc_commit(ctx->chan, phys, virt, n,
				     q->sizeimage, q->bytesperline);
}

/*
 * Drain ready frames in zero-copy mode: the VPSS has already written NV12 into
 * one of our committed buffers; RecvVpssFrmBuf tells us which (by phys). We must
 * hand back THAT vb2 buffer, not "the next free one". The VPSS returns buffers
 * in commit order and apps re-QBUF in dequeue order, so the m2m dst FIFO head
 * normally already is that buffer — verified by phys; a mismatch is logged (it
 * would mean the FIFO assumption broke and needs a per-phys lookup, KI-003).
 * No memcpy. The buffer is returned to the VPSS pool later, on re-QBUF.
 */
static int zc_drain_ready(struct hi_dec_ctx *ctx, bool *eos)
{
	struct vb2_v4l2_buffer *dst;
	struct hi_vdec_frame fr;
	int n = 0;

	while ((dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx)) != NULL) {
		u32 hphys = (u32)vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
		u64 pts;
		u32 usec;

		if (hi_vdec_hal_zc_recv(ctx->chan, &fr) != 0)
			break;				/* nothing ready */
		if (fr.last)
			break;
		if (fr.phys_y != hphys) {
			dprintk(ctx->dev,
				"zc: FIFO mismatch head=0x%08x frame=0x%08x\n",
				hphys, fr.phys_y);
			hi_vdec_hal_zc_release(ctx->chan, &fr);
			break;
		}
		pts = fr.pts;
		usec = do_div(pts, 1000000);
		vb2_set_plane_payload(&dst->vb2_buf, 0,
				      ctx->q_data[Q_DST].sizeimage);
		dst->timestamp.tv_sec  = pts;
		dst->timestamp.tv_usec = usec;
		dst->sequence = ctx->q_data[Q_DST].sequence++;
		dst->field = V4L2_FIELD_NONE;
		dst->flags &= ~V4L2_BUF_FLAG_LAST;
		/* remember the frame so re-QBUF can release THIS buffer to the VPSS */
		ctx->zc_frame[dst->vb2_buf.index] = fr;
		ctx->zc_out[dst->vb2_buf.index] = true;
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
		n++;
	}
	return n;
}

static inline int drain_ready(struct hi_dec_ctx *ctx, bool *eos)
{
	return ctx->zc ? zc_drain_ready(ctx, eos) : dec_drain_ready(ctx, eos);
}

/*
 * device_run (T5): FEED-ONLY. Pushes one input AU into the decoder's ES ring
 * (non-blocking — feed_es just enqueues; the VFMW decodes asynchronously) and
 * returns immediately. Frame capture/de-tile/buf_done is done by cap_thread,
 * fully decoupled from this OUTPUT cadence. This is what lets GStreamer run at
 * the decode rate while ffmpeg (which feeds the whole stream then stops without
 * DEC_CMD_STOP) still gets every frame — the thread drains regardless of feed.
 *
 * One AU per run keeps job_finish recursion bounded by the queued src count and
 * avoids overflowing the ES ring.
 */
static void device_run(void *priv)
{
	struct hi_dec_ctx *ctx = priv;
	struct hi_dec_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src;

	/*
	 * Zero-copy: commit the CAPTURE buffers to the VPSS and ChanStart on the
	 * first run (deferred from start_streaming because the buffers must be
	 * allocated AND the channel must exist with USER_ALLOC set before commit).
	 */
	if (ctx->zc && !ctx->zc_started) {
		if (zc_commit_buffers(ctx) == 0 &&
		    hi_vdec_hal_start(ctx->chan) == 0) {
			ctx->zc_started = true;
			dprintk(dev, "zc: committed CAPTURE buffers + ChanStart\n");
		} else {
			dprintk(dev, "zc: commit/start failed\n");
			v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
			return;
		}
	}

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	if (src) {
		void *es = vb2_plane_vaddr(&src->vb2_buf, 0);
		u32 es_len = vb2_get_plane_payload(&src->vb2_buf, 0);
		u64 pts = (u64)src->timestamp.tv_sec * 1000000ULL +
			  src->timestamp.tv_usec;

		hi_vdec_hal_feed_es(ctx->chan, es, es_len, pts, true);
		src->sequence = ctx->q_data[Q_SRC].sequence++;
		if (ctx->q_data[Q_SRC].sequence <= 5 ||
		    ctx->q_data[Q_SRC].sequence % 60 == 0)
			pr_debug("HI-DBG: feed_es #%u len=%u inflight=%d\n",
				ctx->q_data[Q_SRC].sequence, es_len,
				atomic_read(&ctx->inflight));
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
		atomic_inc(&ctx->inflight);
	}

	/* Frames may now be decoding — let the capture thread drain them. */
	wake_up_interruptible(&ctx->cap_wq);
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
}

/*
 * Capture thread (T5): drains decoded frames into CAPTURE buffers, decoupled
 * from feed. Parks on cap_wq when idle (no frames in flight, not draining) — no
 * busy polling. While frames are decoding it polls RecvFrmBuf at ~1-2ms (the
 * HAL exposes no frame-ready IRQ). The de-tile (copy_frame_to_buf, ~30ms) runs
 * here, holding NO lock — dev_mutex (the vb2 queue lock) is never taken by this
 * thread, so qbuf/dqbuf are never blocked by a de-tile. Buffer-list ops
 * (v4l2_m2m_*_dst_buf) and buf_done use vb2/m2m internal spinlocks only.
 *
 * Lifecycle: created when CAPTURE+OUTPUT are both streaming (a channel exists),
 * torn down with kthread_stop() at stop_streaming BEFORE the buffer flush — so
 * the flush never races a frame still being delivered.
 */
static int hi_dec_cap_thread(void *data)
{
	struct hi_dec_ctx *ctx = data;
	struct hi_dec_dev *dev = ctx->dev;
	int idle = 0, drain_empty = 0;

	while (!kthread_should_stop()) {
		/* Park until there is capture work (frames in flight or an EOS
		 * drain) or a stop request. No timeout: zero CPU when idle. */
		wait_event_interruptible(ctx->cap_wq,
			kthread_should_stop() ||
			(ctx->cap_streaming &&
			 (atomic_read(&ctx->inflight) > 0 || ctx->draining)));
		if (kthread_should_stop())
			break;
		if (!ctx->cap_streaming)
			continue;

		idle = 0;
		drain_empty = 0;
		while (!kthread_should_stop() && ctx->cap_streaming &&
		       (atomic_read(&ctx->inflight) > 0 || ctx->draining)) {
			bool eos = false;
			int n;

			if (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) < 1) {
				/* No CAPTURE buffer; wait for the app to queue one. */
				wait_event_interruptible_timeout(ctx->cap_wq,
					kthread_should_stop() || !ctx->cap_streaming ||
					v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) >= 1,
					msecs_to_jiffies(50));
				continue;
			}

			n = drain_ready(ctx, &eos);
			if (n > 0) {
				if (atomic_sub_return(n, &ctx->inflight) < 0)
					atomic_set(&ctx->inflight, 0);
				idle = drain_empty = 0;
				ctx->reorder_fill = false;	/* output flowing */
				ctx->last_out_jiffies = jiffies;	/* watchdog */
				/* Headroom freed — resume feeding (back-pressure). */
				v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
				continue;
			}
			if (eos) {		/* EOS sentinel: drain emitted LAST */
				atomic_set(&ctx->inflight, 0);
				break;
			}
			if (ctx->draining) {
				/* Tail frames in the VPSS are unreachable via direct
				 * RecvFrm (KI-002); a quiet window means end-of-stream:
				 * emit the LAST buffer + EOS event ourselves. */
				if (++drain_empty >= HI_DEC_DRAIN_EMPTY_MAX) {
					struct vb2_v4l2_buffer *dst =
						v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
					if (dst)
						dec_emit_eos(ctx, dst);
					atomic_set(&ctx->inflight, 0);
					break;
				}
			} else {
				++idle;
				/* No output for longer than normal decode latency while
				 * frames are in flight => reorder-fill (deep B-reorder or
				 * startup): the decoder needs MORE input before it can emit
				 * the first reordered frame. Lift the cap so the feed isn't
				 * deadlocked, and re-run the scheduler. Cleared on next output. */
				if (idle == HI_DEC_STARVE_MAX &&
				    atomic_read(&ctx->inflight) > 0 &&
				    !ctx->reorder_fill) {
					ctx->reorder_fill = true;
					v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
				}
				/* Watchdog: no output for too long while input is still
				 * pending (cap already lifted) => the decoder is wedged on
				 * bad/unsupported ES (it can't make progress). Don't hang the
				 * consumer's dqbuf/poll forever — signal EOS so it ends or
				 * errors gracefully. Time-based (jiffies), so it survives the
				 * poll back-off below and never trips on a merely-slow stream
				 * (last_out_jiffies is refreshed on every delivered frame). */
				if (atomic_read(&ctx->inflight) > 0 &&
				    time_after(jiffies, ctx->last_out_jiffies +
					       msecs_to_jiffies(HI_DEC_WATCHDOG_MS))) {
					struct vb2_v4l2_buffer *dst =
						v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

					dprintk(dev, "watchdog: decode stalled ~%ums, EOS\n",
						jiffies_to_msecs(jiffies -
								 ctx->last_out_jiffies));
					if (dst)
						dec_emit_eos(ctx, dst);
					atomic_set(&ctx->inflight, 0);
					ctx->reorder_fill = false;
					break;
				}
			}
			/* Fast poll while a frame may be imminent (reorder/startup); back
			 * off after a long idle to spare CPU during a stall — the watchdog
			 * still fires on wall-clock. */
			if (idle < HI_DEC_CAP_IDLE_MAX)
				usleep_range(HI_DEC_CAP_POLL_US_MIN,
					     HI_DEC_CAP_POLL_US_MAX);
			else
				msleep(20);
		}
	}
	return 0;
}

/* Start the capture thread once OUTPUT (channel) and CAPTURE are both up. */
static void hi_dec_cap_thread_start(struct hi_dec_ctx *ctx)
{
	if (ctx->cap_thread || !ctx->chan || !ctx->cap_streaming)
		return;
	ctx->cap_thread = kthread_run(hi_dec_cap_thread, ctx, "hi-vdec-cap");
	if (IS_ERR(ctx->cap_thread)) {
		dprintk(ctx->dev, "cap_thread create failed (%ld)\n",
			PTR_ERR(ctx->cap_thread));
		ctx->cap_thread = NULL;
	}
}

/* Stop the capture thread (blocks until it exits) — call BEFORE flushing
 * CAPTURE buffers so the thread can never race the flush. */
static void hi_dec_cap_thread_stop(struct hi_dec_ctx *ctx)
{
	if (ctx->cap_thread) {
		kthread_stop(ctx->cap_thread);
		ctx->cap_thread = NULL;
	}
}

/* Adaptive back-pressure cap (see HI_DEC_MAX_INFLIGHT*): the larger reorder cap
 * only while the capture thread has flagged a reorder-fill stall. */
static u32 hi_dec_inflight_cap(struct hi_dec_ctx *ctx)
{
	return ctx->reorder_fill ? HI_DEC_MAX_INFLIGHT_REORDER
				 : HI_DEC_MAX_INFLIGHT;
}

static int job_ready(void *priv)
{
	struct hi_dec_ctx *ctx = priv;

	/*
	 * device_run is FEED-ONLY now (T5) — it consumes an OUTPUT (coded) buffer
	 * and does not touch CAPTURE. So a job needs input AND headroom in the
	 * decoder (back-pressure): if too many fed frames are still undrained, hold
	 * off feeding so the decoder cannot race far ahead of the consumer (which
	 * would strand a backlog that single-batch drainers like ffmpeg can't pull).
	 * The cap is per codec — HEVC's deep B-pyramid reorder needs a larger one or
	 * the feed deadlocks against the reorder fill. cap_thread re-runs the
	 * scheduler as it drains, so feeding resumes.
	 */
	if (atomic_read(&ctx->inflight) >= hi_dec_inflight_cap(ctx))
		return 0;
	return v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) >= 1;
}

static void job_abort(void *priv)
{
	struct hi_dec_ctx *ctx = priv;

	ctx->aborting = 1;
}

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_ready	= job_ready,
	.job_abort	= job_abort,
};

/* -------------------------------------------------------------------------- */
/* ioctls                                                                     */
/* -------------------------------------------------------------------------- */

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	pr_debug("HI-DBG: QUERYCAP\n");
	strlcpy(cap->driver, HI_DEC_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "Hisilicon VDEC", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", HI_DEC_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 flag)
{
	const struct hi_v4l2_fmt *fmt = hi_v4l2_enum_fmt(f->index, flag);

	pr_debug("HI-DBG: ENUM_FMT %s idx=%u -> %s 0x%08x\n",
		flag == HI_V4L2_CODED ? "OUT" : "CAP", f->index,
		fmt ? "ok" : "EINVAL", fmt ? fmt->fourcc : 0);
	if (!fmt)
		return -EINVAL;
	f->pixelformat = fmt->fourcc;
	if (flag == HI_V4L2_CODED)
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int vidioc_enum_fmt_cap(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, HI_V4L2_RAW);
}

static int vidioc_enum_fmt_out(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, HI_V4L2_CODED);
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hi_dec_ctx *ctx = file2ctx(file);
	struct hi_dec_q_data *q = get_q_data(ctx, f->type);

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
	pr_debug("HI-DBG: G_FMT %s -> 0x%08x %ux%u\n",
		V4L2_TYPE_IS_OUTPUT(f->type) ? "OUT" : "CAP",
		f->fmt.pix.pixelformat, f->fmt.pix.width, f->fmt.pix.height);
	return 0;
}

static int vidioc_try_fmt_out(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	const struct hi_v4l2_fmt *fmt;

	pr_debug("HI-DBG: TRY_FMT_OUT req=0x%08x %ux%u\n",
		f->fmt.pix.pixelformat, f->fmt.pix.width, f->fmt.pix.height);
	fmt = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_CODED);
	if (!fmt) {
		fmt = hi_v4l2_enum_fmt(0, HI_V4L2_CODED);
		f->fmt.pix.pixelformat = fmt->fourcc;
	}
	f->fmt.pix.width  = clamp_t(u32, f->fmt.pix.width,  16, HI_DEC_MAX_W);
	f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 16, HI_DEC_MAX_H);
	f->fmt.pix.field  = V4L2_FIELD_NONE;
	/* coded buffer: a single bytestream plane sized generously */
	if (!f->fmt.pix.sizeimage)
		f->fmt.pix.sizeimage = HI_DEC_ES_BUF_SIZE;
	f->fmt.pix.bytesperline = 0;
	/* OUTPUT colorimetry is app-defined; default it when left unset */
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
	struct hi_dec_ctx *ctx = file2ctx(file);
	const struct hi_v4l2_fmt *fmt;
	u32 bpl;

	pr_debug("HI-DBG: TRY_FMT_CAP req=0x%08x %ux%u\n",
		f->fmt.pix.pixelformat, f->fmt.pix.width, f->fmt.pix.height);
	fmt = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_RAW);
	if (!fmt) {
		fmt = hi_v4l2_find_fmt(V4L2_PIX_FMT_NV12, HI_V4L2_RAW);
		f->fmt.pix.pixelformat = fmt->fourcc;
	}
	f->fmt.pix.width  = clamp_t(u32, f->fmt.pix.width,  16, HI_DEC_MAX_W);
	f->fmt.pix.height = clamp_t(u32, f->fmt.pix.height, 16, HI_DEC_MAX_H);
	f->fmt.pix.field  = V4L2_FIELD_NONE;
	f->fmt.pix.sizeimage = hi_v4l2_raw_sizeimage(f->fmt.pix.pixelformat,
						     f->fmt.pix.width,
						     f->fmt.pix.height, &bpl);
	f->fmt.pix.bytesperline = bpl;
	/* CAPTURE colorimetry is fixed by the OUTPUT (coded) side */
	f->fmt.pix.colorspace   = ctx->colorspace;
	f->fmt.pix.ycbcr_enc    = ctx->ycbcr_enc;
	f->fmt.pix.quantization = ctx->quantization;
	f->fmt.pix.xfer_func    = ctx->xfer_func;
	return 0;
}

static int vidioc_s_fmt_out(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct hi_dec_ctx *ctx = file2ctx(file);
	struct hi_dec_q_data *q = &ctx->q_data[Q_SRC];
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;
	ret = vidioc_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	q->fmt          = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_CODED);
	q->width        = f->fmt.pix.width;
	q->height       = f->fmt.pix.height;
	q->sizeimage    = f->fmt.pix.sizeimage;
	q->bytesperline = 0;

	/* latch the app-provided colorimetry; CAPTURE inherits it */
	ctx->colorspace   = f->fmt.pix.colorspace;
	ctx->ycbcr_enc    = f->fmt.pix.ycbcr_enc;
	ctx->quantization = f->fmt.pix.quantization;
	ctx->xfer_func    = f->fmt.pix.xfer_func;

	/* default the CAPTURE geometry to match the coded resolution */
	ctx->q_data[Q_DST].width  = q->width;
	ctx->q_data[Q_DST].height = q->height;
	ctx->q_data[Q_DST].sizeimage =
		hi_v4l2_raw_sizeimage(ctx->q_data[Q_DST].fmt->fourcc,
				      q->width, q->height,
				      &ctx->q_data[Q_DST].bytesperline);
	pr_debug("HI-DBG: S_FMT_OUT 0x%08x %ux%u sz=%u fmt=%p\n",
		f->fmt.pix.pixelformat, q->width, q->height, q->sizeimage, q->fmt);
	return 0;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct hi_dec_ctx *ctx = file2ctx(file);
	struct hi_dec_q_data *q = &ctx->q_data[Q_DST];
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;
	ret = vidioc_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	q->fmt          = hi_v4l2_find_fmt(f->fmt.pix.pixelformat, HI_V4L2_RAW);
	q->width        = f->fmt.pix.width;
	q->height       = f->fmt.pix.height;
	q->sizeimage    = f->fmt.pix.sizeimage;
	q->bytesperline = f->fmt.pix.bytesperline;
	pr_debug("HI-DBG: S_FMT_CAP 0x%08x %ux%u sz=%u\n",
		f->fmt.pix.pixelformat, q->width, q->height, q->sizeimage);
	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static int vidioc_try_decoder_cmd(struct file *file, void *priv,
				  struct v4l2_decoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_DEC_CMD_STOP && cmd->cmd != V4L2_DEC_CMD_START)
		return -EINVAL;
	return 0;
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *cmd)
{
	struct hi_dec_ctx *ctx = file2ctx(file);

	if (cmd->cmd != V4L2_DEC_CMD_STOP && cmd->cmd != V4L2_DEC_CMD_START)
		return -EINVAL;
	if (cmd->cmd == V4L2_DEC_CMD_START) {
		ctx->draining = false;
		return 0;
	}
	/* STOP: tell the decoder no more input and drain the DPB. The drain runs
	 * as capture-only jobs (job_ready/device_run) until the DPB is empty, then
	 * the last CAPTURE buffer carries V4L2_BUF_FLAG_LAST + a V4L2_EVENT_EOS. */
	if (ctx->chan) {
		hi_vdec_hal_set_eos(ctx->chan);
		ctx->draining = true;
		ctx->drain_empty = 0;
		dprintk(ctx->dev, "DEC_CMD_STOP: drain start src=%d dst=%d\n",
			v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx),
			v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx));
		/* Flush any still-queued input, then let cap_thread drain the DPB
		 * and emit the LAST buffer + EOS event. */
		v4l2_m2m_try_schedule(ctx->fh.m2m_ctx);
		wake_up_interruptible(&ctx->cap_wq);
	}
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	return hi_v4l2_fill_framesize(fsize, HI_DEC_MAX_W, HI_DEC_MAX_H);
}

static const struct v4l2_ioctl_ops hi_dec_ioctl_ops = {
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

	.vidioc_decoder_cmd		= vidioc_decoder_cmd,
	.vidioc_try_decoder_cmd		= vidioc_try_decoder_cmd,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,
	.vidioc_subscribe_event		= vidioc_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* -------------------------------------------------------------------------- */
/* vb2 queue ops                                                              */
/* -------------------------------------------------------------------------- */

static int hi_dec_queue_setup(struct vb2_queue *vq, const void *parg,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], void *alloc_ctxs[])
{
	struct hi_dec_ctx *ctx = vb2_get_drv_priv(vq);
	struct hi_dec_q_data *q = get_q_data(ctx, vq->type);
	const struct v4l2_format *fmt = parg;
	u32 size = q->sizeimage;

	if (fmt) {
		/* VIDIOC_CREATE_BUFS: a plane too small for the format is invalid */
		if (fmt->fmt.pix.sizeimage < size)
			return -EINVAL;
		size = fmt->fmt.pix.sizeimage;
	}
	if (!size)
		size = HI_DEC_ES_BUF_SIZE;

	if (*nbuffers < 2)
		*nbuffers = 2;
	*nplanes = 1;
	sizes[0] = size;
	pr_debug("HI-DBG: queue_setup %s n=%u size=%u\n",
		V4L2_TYPE_IS_OUTPUT(vq->type) ? "OUTPUT" : "CAPTURE",
		*nbuffers, sizes[0]);
	/* CAPTURE is dma-contig so decoded frames are dma-buf exportable
	 * (VIDIOC_EXPBUF) -> standard zero-copy to encoder/GPU. OUTPUT (coded
	 * ES) stays vmalloc (CPU-fed, context-less). */
	if (!V4L2_TYPE_IS_OUTPUT(vq->type))
		alloc_ctxs[0] = ctx->dev->alloc_ctx;
	return 0;
}

static int hi_dec_buf_prepare(struct vb2_buffer *vb)
{
	struct hi_dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct hi_dec_q_data *q = get_q_data(ctx, vb->vb2_queue->type);

	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vb2_plane_size(vb, 0) < q->sizeimage)
			return -EINVAL;
		vb2_set_plane_payload(vb, 0, q->sizeimage);
	}
	return 0;
}

static void hi_dec_buf_queue(struct vb2_buffer *vb)
{
	struct hi_dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	/*
	 * Zero-copy: a CAPTURE buffer coming back from the app must be returned to
	 * the VPSS pool before it is eligible to be filled again. Only buffers we
	 * previously handed out (zc_out) are released — the initial QBUFs are
	 * already in the pool from zc_commit_buffers().
	 */
	if (ctx->zc && ctx->zc_started &&
	    !V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    ctx->zc_out[vb->index]) {
		hi_vdec_hal_zc_release(ctx->chan, &ctx->zc_frame[vb->index]);
		ctx->zc_out[vb->index] = false;
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);

	/* A freshly queued CAPTURE buffer may unblock cap_thread (it parks when no
	 * output buffer is available to drain into). */
	if (!V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		wake_up_interruptible(&ctx->cap_wq);
}

static int hi_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hi_dec_ctx *ctx = vb2_get_drv_priv(q);
	struct hi_dec_q_data *qs = &ctx->q_data[Q_SRC];
	int ret;

	get_q_data(ctx, q->type)->sequence = 0;
	ctx->draining = false;

	pr_debug("HI-DBG: start_streaming %s count=%u chan=%p\n",
		V4L2_TYPE_IS_OUTPUT(q->type) ? "OUTPUT" : "CAPTURE",
		count, ctx->chan);

	/* CAPTURE side up: enable + (re)start the capture thread. It starts now if
	 * the channel already exists (OUTPUT streaming); otherwise when OUTPUT
	 * starts below. */
	if (!V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->cap_streaming = true;
		hi_dec_cap_thread_start(ctx);
		wake_up_interruptible(&ctx->cap_wq);
		return 0;
	}

	/* Bring the decode channel up once, when the OUTPUT side starts. */
	if (ctx->chan)
		return 0;

	atomic_set(&ctx->inflight, 0);
	ctx->reorder_fill = false;
	ctx->last_out_jiffies = jiffies;	/* watchdog baseline */
	ctx->zc = zc;			/* latch the module param for this instance */
	if (ctx->zc) {
		/*
		 * Zero-copy: build a real VPSS HD/MASTER port with USER_ALLOC set
		 * before CreatePort. ChanStart + buffer commit are deferred to the
		 * first device_run() (CAPTURE buffers must be allocated first).
		 */
		ctx->chan = hi_vdec_hal_zc_create(qs->fmt, qs->width, qs->height,
						  HI_DEC_ES_BUF_SIZE);
		if (IS_ERR(ctx->chan)) {
			ret = PTR_ERR(ctx->chan);
			ctx->chan = NULL;
			return ret;
		}
		ctx->zc_started = false;
		hi_dec_cap_thread_start(ctx);
		return 0;
	}

	if (win) {
		/*
		 * VO virtual-window path: VPSS-managed buffers, the window drives
		 * the VPSS to produce LINEAR NV12. recv/release dispatch on the
		 * window handle; the standard memcpy drain (dec_drain_ready) is
		 * reused. ChanStart now, like the default path.
		 */
		ctx->chan = hi_vdec_hal_win_create(qs->fmt, qs->width, qs->height,
						   HI_DEC_ES_BUF_SIZE);
		if (IS_ERR(ctx->chan)) {
			ret = PTR_ERR(ctx->chan);
			ctx->chan = NULL;
			return ret;
		}
		ret = hi_vdec_hal_start(ctx->chan);
		if (ret == 0)
			hi_dec_cap_thread_start(ctx);
		return ret;
	}

	if (vp) {
		/*
		 * Variant B: VPSS-managed port, no window. VPSS allocates+processes
		 * its own linear NV12; recv via RecvVpssFrmBuf + memcpy (dispatched
		 * on the chan's vpss_recv flag). Standard memcpy drain reused.
		 */
		ctx->chan = hi_vdec_hal_vp_create(qs->fmt, qs->width, qs->height,
						  HI_DEC_ES_BUF_SIZE);
		if (IS_ERR(ctx->chan)) {
			ret = PTR_ERR(ctx->chan);
			ctx->chan = NULL;
			return ret;
		}
		ret = hi_vdec_hal_start(ctx->chan);
		if (ret == 0)
			hi_dec_cap_thread_start(ctx);
		return ret;
	}

	/* SW de-tile uses the default VFMW-direct path; enable its prerequisites
	 * (map_frm + maskcmp) on the vendor VDEC before the channel starts. */
	if (detile)
		hi_vdec_hal_enable_detile();

	ctx->chan = hi_vdec_hal_create(qs->fmt, qs->width, qs->height,
				       HI_DEC_ES_BUF_SIZE);
	if (IS_ERR(ctx->chan)) {
		ret = PTR_ERR(ctx->chan);
		ctx->chan = NULL;
		pr_debug("HI-DBG: hal_create FAILED %d\n", ret);
		return ret;
	}
	ret = hi_vdec_hal_start(ctx->chan);
	pr_debug("HI-DBG: hal_create ok chan=%p, hal_start ret=%d\n",
		ctx->chan, ret);
	if (ret == 0)
		hi_dec_cap_thread_start(ctx);
	return ret;
}

static void hi_dec_stop_streaming(struct vb2_queue *q)
{
	struct hi_dec_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;

	/*
	 * Stop the capture thread FIRST (kthread_stop blocks until it exits) so it
	 * can never be touching a CAPTURE buffer while we flush below. It does not
	 * take dev_mutex, so stopping it from under the queue lock is deadlock-free.
	 * It is recreated on the next start_streaming.
	 */
	hi_dec_cap_thread_stop(ctx);
	if (!V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->cap_streaming = false;

	ctx->draining = false;
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
		hi_vdec_hal_destroy(ctx->chan);
		ctx->chan = NULL;
		ctx->zc_started = false;
		memset(ctx->zc_out, 0, sizeof(ctx->zc_out));
	}
}

static struct vb2_ops hi_dec_qops = {
	.queue_setup	 = hi_dec_queue_setup,
	.buf_prepare	 = hi_dec_buf_prepare,
	.buf_queue	 = hi_dec_buf_queue,
	.start_streaming = hi_dec_start_streaming,
	.stop_streaming	 = hi_dec_stop_streaming,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct hi_dec_ctx *ctx = priv;
	int ret;

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes	= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	src_vq->ops		= &hi_dec_qops;
	src_vq->mem_ops		= &vb2_vmalloc_memops;
	src_vq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock		= &ctx->dev->dev_mutex;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes	= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops		= &hi_dec_qops;
	/*
	 * CAPTURE is CACHED (vmalloc), not write-combine (dma-contig). The frame is
	 * only ever CPU-written (de-tile copies VFMW->vb2; no device DMAs into it)
	 * and CPU-read (GStreamer/FFmpeg, and Kodi's glTexImage2D upload — Mali has
	 * no dma_buf_import here, so the client reads the buffer with the CPU).
	 * Write-combine made that upload read ~140 ms/frame @1080p (uncached) and
	 * capped Kodi at ~5 fps; a cached mapping makes it a normal cached read.
	 * SMP cache coherency covers the kthread-writer / client-reader hand-off.
	 * (dma-contig was only for dma-buf export, which this SoC's GPU can't use.)
	 */
	dst_vq->mem_ops		= &vb2_vmalloc_memops;
	dst_vq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	return vb2_queue_init(dst_vq);
}

/* -------------------------------------------------------------------------- */
/* file ops                                                                   */
/* -------------------------------------------------------------------------- */

static void init_q_defaults(struct hi_dec_ctx *ctx)
{
	struct hi_dec_q_data *src = &ctx->q_data[Q_SRC];
	struct hi_dec_q_data *dst = &ctx->q_data[Q_DST];

	src->fmt        = hi_v4l2_enum_fmt(0, HI_V4L2_CODED);
	src->width      = HI_DEC_DEF_W;
	src->height     = HI_DEC_DEF_H;
	src->sizeimage  = HI_DEC_ES_BUF_SIZE;

	dst->fmt        = hi_v4l2_find_fmt(V4L2_PIX_FMT_NV12, HI_V4L2_RAW);
	dst->width      = HI_DEC_DEF_W;
	dst->height     = HI_DEC_DEF_H;
	dst->sizeimage  = hi_v4l2_raw_sizeimage(dst->fmt->fourcc,
						dst->width, dst->height,
						&dst->bytesperline);

	ctx->colorspace   = V4L2_COLORSPACE_REC709;
	ctx->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	ctx->quantization = V4L2_QUANTIZATION_DEFAULT;
	ctx->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
}

static int hi_dec_open(struct file *file)
{
	struct hi_dec_dev *dev = video_drvdata(file);
	struct hi_dec_ctx *ctx;
	int ret = 0;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}
	ctx->dev = dev;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	init_q_defaults(ctx);
	init_waitqueue_head(&ctx->cap_wq);
	atomic_set(&ctx->inflight, 0);

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto unlock;
	}
	/* Allow the m2m scheduler to run a job with an empty OUTPUT (input) queue
	 * so the EOS drain can flush the decoder DPB without more input; job_ready
	 * still gates this on ctx->draining. */
	v4l2_m2m_set_src_buffered(ctx->fh.m2m_ctx, true);

	v4l2_fh_add(&ctx->fh);
	atomic_inc(&dev->num_inst);
	dprintk(dev, "instance %p opened\n", ctx);
unlock:
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int hi_dec_release(struct file *file)
{
	struct hi_dec_dev *dev = video_drvdata(file);
	struct hi_dec_ctx *ctx = file2ctx(file);

	pr_debug("HI-DBG: RELEASE instance %p chan=%p\n", ctx, ctx->chan);

	/* v4l2_m2m_ctx_release() drives streamoff -> stop_streaming, which stops
	 * the capture thread; stop it explicitly too in case the app closed without
	 * a clean STREAMOFF. kthread_stop() is a no-op if already stopped. */
	hi_dec_cap_thread_stop(ctx);

	mutex_lock(&dev->dev_mutex);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	if (ctx->chan)
		hi_vdec_hal_destroy(ctx->chan);
	mutex_unlock(&dev->dev_mutex);

	kfree(ctx);
	atomic_dec(&dev->num_inst);
	return 0;
}

static const struct v4l2_file_operations hi_dec_fops = {
	.owner		= THIS_MODULE,
	.open		= hi_dec_open,
	.release	= hi_dec_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct video_device hi_dec_videodev = {
	.name		= HI_DEC_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &hi_dec_fops,
	.ioctl_ops	= &hi_dec_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
};

/* -------------------------------------------------------------------------- */
/* platform driver                                                            */
/* -------------------------------------------------------------------------- */

static void hi_dec_advertise_caps(struct hi_dec_dev *dev)
{
	int n = hi_v4l2_probe_hw_caps();

	if (n < 0)
		v4l2_info(&dev->v4l2_dev,
			  "advertising full coded table (GetCap not compiled in)\n");
	else
		v4l2_info(&dev->v4l2_dev,
			  "%d coded formats enabled by HW caps\n", n);
}

static int hi_dec_probe(struct platform_device *pdev)
{
	struct hi_dec_dev *dev;
	struct video_device *vfd;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->pdev = pdev;
	mutex_init(&dev->dev_mutex);
	atomic_set(&dev->num_inst, 0);

	/*
	 * Open the VDEC subsystem here (refcounted; coexists with AVServer). The
	 * msp boot init (hi_init.c) already did VDEC_DRV_ModInit, and our
	 * late_initcall_sync (hi_v4l2_core.c) runs after it, so the VFMW/VPSS
	 * functions the open needs are already registered.
	 */
	ret = hi_vdec_hal_global_init();
	if (ret)
		return ret;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		goto err_hal;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_v4l2;
	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		ret = PTR_ERR(dev->alloc_ctx);
		goto err_v4l2;
	}

	hi_dec_advertise_caps(dev);

	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		ret = PTR_ERR(dev->m2m_dev);
		goto err_ctx;
	}

	dev->vfd = hi_dec_videodev;
	vfd = &dev->vfd;
	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret)
		goto err_m2m;

	video_set_drvdata(vfd, dev);
	platform_set_drvdata(pdev, dev);
	v4l2_info(&dev->v4l2_dev, "decoder registered as /dev/video%d\n",
		  vfd->num);
	return 0;

err_m2m:
	v4l2_m2m_release(dev->m2m_dev);
err_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
err_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_hal:
	hi_vdec_hal_global_exit();
	return ret;
}

static int hi_dec_remove(struct platform_device *pdev)
{
	struct hi_dec_dev *dev = platform_get_drvdata(pdev);

	video_unregister_device(&dev->vfd);
	v4l2_m2m_release(dev->m2m_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	v4l2_device_unregister(&dev->v4l2_dev);
	hi_vdec_hal_global_exit();
	return 0;
}

static void hi_dec_pdev_release(struct device *dev) {}

static struct platform_device hi_dec_pdev = {
	.name		= HI_DEC_NAME,
	.dev.release	= hi_dec_pdev_release,
};

/* CMA-backed device used for vb2-dma-contig; reused by the zero-copy probe. */
struct device *hi_dec_dma_dev(void)
{
	return &hi_dec_pdev.dev;
}

static struct platform_driver hi_dec_pdrv = {
	.probe		= hi_dec_probe,
	.remove		= hi_dec_remove,
	.driver		= {
		.name	= HI_DEC_NAME,
	},
};

int hi_dec_register(void)
{
	int ret = platform_device_register(&hi_dec_pdev);

	if (ret)
		return ret;
	ret = platform_driver_register(&hi_dec_pdrv);
	if (ret)
		platform_device_unregister(&hi_dec_pdev);
	return ret;
}

void hi_dec_unregister(void)
{
	platform_driver_unregister(&hi_dec_pdrv);
	platform_device_unregister(&hi_dec_pdev);
}
