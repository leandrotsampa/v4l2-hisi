// SPDX-License-Identifier: GPL-2.0
/*
 * HAL over HI_DRV_VENC_* (venc_v2.0). See hi_venc_hal.h.
 *
 * Lifecycle mirrors AVServer/encoder.c and sample/venc:
 *   Init -> GetDefaultAttr -> (tune attr) -> Create -> Start
 *   loop: QueueFrame(raw, phys) ; AcquireStream -> use -> ReleaseStream ; DequeueFrame
 *   Stop -> Destroy -> DeInit
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include "hi_venc_hal.h"

struct hi_venc_chan {
	HI_HANDLE	hVenc;
	bool		started;
};

static DEFINE_MUTEX(g_lock);
static int g_init_refcnt;

int hi_venc_hal_global_init(void)
{
	int ret = 0;

	mutex_lock(&g_lock);
	if (g_init_refcnt == 0) {
		ret = HI_DRV_VENC_Init();
		if (ret != HI_SUCCESS) {
			pr_err("hi-v4l2: HI_DRV_VENC_Init failed (0x%x)\n", ret);
			ret = -EIO;
			goto out;
		}
	}
	g_init_refcnt++;
out:
	mutex_unlock(&g_lock);
	return ret;
}

void hi_venc_hal_global_exit(void)
{
	mutex_lock(&g_lock);
	if (--g_init_refcnt == 0)
		HI_DRV_VENC_DeInit();
	if (g_init_refcnt < 0)
		g_init_refcnt = 0;
	mutex_unlock(&g_lock);
}

static HI_UNF_VCODEC_CAP_LEVEL_E cap_level_for(u32 w, u32 h)
{
	u32 m = max(w, h);

	if (m > 2160)
		return HI_UNF_VCODEC_CAP_LEVEL_4096x2160;
	if (m > 1280)
		return HI_UNF_VCODEC_CAP_LEVEL_FULLHD;
	if (m > 720)
		return HI_UNF_VCODEC_CAP_LEVEL_720P;
	return HI_UNF_VCODEC_CAP_LEVEL_D1;
}

struct hi_venc_chan *hi_venc_hal_create(const struct hi_v4l2_fmt *coded,
					const struct hi_venc_params *p)
{
	struct hi_venc_chan *c;
	HI_UNF_VENC_CHN_ATTR_S attr;
	int ret;

	if (!coded || !(coded->flags & HI_V4L2_ENC) || !p)
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	ret = HI_DRV_VENC_GetDefaultAttr(&attr);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: VENC GetDefaultAttr failed (0x%x)\n", ret);
		goto err_free;
	}

	attr.enVencType    = coded->vcodec;
	attr.enCapLevel    = cap_level_for(p->width, p->height);
	attr.enVencProfile = p->profile;
	/*
	 * JPEG/MJPEG need a valid quality level (1..99); GetDefaultAttr leaves
	 * u32Qlevel=0, which CheckPrivateAttr rejects for JPEG (drv_venc.c:591).
	 * Harmless for H264/HEVC (not range-checked there).
	 */
	attr.u32Qlevel     = 90;
	attr.u32Width      = ALIGN(p->width, 4);
	attr.u32Height     = ALIGN(p->height, 4);
	/*
	 * HI_DRV_VENC's u32TargetBitRate is in bps, NOT kbps: GetDefaultAttr sets
	 * 4*1024*1024 and CheckPrivateAttr bounds it by HI_VENC_MIN_bps/MAX_bps
	 * (drv_venc.c:625,2898). V4L2_CID_MPEG_VIDEO_BITRATE is also bps, so pass
	 * it straight through. Dividing by 1000 (an earlier wrong RE note) yielded
	 * 4000 bps for a 4 Mbps default -> below MIN -> VENC Create 0x801d0008.
	 */
	if (p->bitrate)
		attr.u32TargetBitRate = p->bitrate;
	if (p->framerate) {
		attr.u32TargetFrmRate = p->framerate;
		attr.u32InputFrmRate  = p->framerate;
	}
	if (p->gop)
		attr.u32Gop = p->gop;
	/*
	 * CheckCommonAttr (drv_venc.c:535) requires
	 *   2*W*H <= u32StrmBufSize <= HI_VENC_MAX_BUF_SIZE (20 MiB).
	 * GetDefaultAttr's value (1843200) is below 2*W*H for HD/FHD, so always
	 * size the stream buffer to the (aligned) resolution instead of only when
	 * it is zero. 2*W*H also covers 4K (~16.6 MiB < 20 MiB).
	 */
	attr.u32StrmBufSize = max(2u * attr.u32Width * attr.u32Height,
				  (u32)(1024 * 1024));
	if (attr.u32StrmBufSize > 20u * 1024 * 1024)
		attr.u32StrmBufSize = 20u * 1024 * 1024 - 4096;

	/* HW-VALIDATE: bOMXChn=HI_FALSE, pfile=NULL (only stored as owner). */
	ret = HI_DRV_VENC_Create(&c->hVenc, &attr, HI_FALSE, NULL);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: VENC Create failed (0x%x)\n", ret);
		goto err_free;
	}
	return c;

err_free:
	kfree(c);
	return ERR_PTR(-EIO);
}

void hi_venc_hal_destroy(struct hi_venc_chan *c)
{
	if (!c)
		return;
	if (c->started)
		hi_venc_hal_stop(c);
	HI_DRV_VENC_Destroy(c->hVenc);
	kfree(c);
}

int hi_venc_hal_start(struct hi_venc_chan *c)
{
	if (c->started)
		return 0;
	if (HI_DRV_VENC_Start(c->hVenc) != HI_SUCCESS)
		return -EIO;
	c->started = true;
	return 0;
}

int hi_venc_hal_stop(struct hi_venc_chan *c)
{
	if (!c->started)
		return 0;
	HI_DRV_VENC_Stop(c->hVenc);
	c->started = false;
	return 0;
}

int hi_venc_hal_request_keyframe(struct hi_venc_chan *c)
{
	return (HI_DRV_VENC_RequestIFrame(c->hVenc) == HI_SUCCESS) ? 0 : -EIO;
}

int hi_venc_hal_queue_frame(struct hi_venc_chan *c, u32 phys_y, u32 ystride,
			    u32 width, u32 height, u64 pts_ms, bool nv21)
{
	HI_UNF_VIDEO_FRAME_INFO_S fi;

	memset(&fi, 0, sizeof(fi));
	fi.u32Width  = width;
	fi.u32Height = height;
	fi.u32Pts    = (u32)pts_ms;
	fi.u32SrcPts = (u32)pts_ms;
	fi.bProgressive = HI_TRUE;
	fi.enFieldMode  = HI_UNF_VIDEO_FIELD_ALL;
	/* NV12 = Y + interleaved CbCr (U first); NV21 = VU first. */
	fi.enVideoFormat = nv21 ? HI_UNF_FORMAT_YUV_SEMIPLANAR_420
				: HI_UNF_FORMAT_YUV_SEMIPLANAR_420_UV;
	fi.stVideoFrameAddr[0].u32YAddr   = phys_y;
	fi.stVideoFrameAddr[0].u32CAddr   = phys_y + ystride * height;
	fi.stVideoFrameAddr[0].u32YStride = ystride;
	fi.stVideoFrameAddr[0].u32CStride = ystride;

	return (HI_DRV_VENC_QueueFrame(c->hVenc, &fi) == HI_SUCCESS) ? 0 : -EIO;
}

void hi_venc_hal_dequeue_frame(struct hi_venc_chan *c)
{
	HI_UNF_VIDEO_FRAME_INFO_S fi;

	/* Best-effort reclaim; the HW is done with the input by now. */
	memset(&fi, 0, sizeof(fi));
	HI_DRV_VENC_DequeueFrame(c->hVenc, &fi);
}

int hi_venc_hal_acquire_stream(struct hi_venc_chan *c, struct hi_venc_stream *s)
{
	memset(s, 0, sizeof(*s));
	if (HI_DRV_VENC_AcquireStream(c->hVenc, &s->raw, 0) != HI_SUCCESS)
		return -EAGAIN;

	s->data      = (void *)(unsigned long)s->raw.VirAddr;
	s->len       = s->raw.u32SlcLen;
	s->pts       = s->raw.u32PtsMs;
	s->frame_end = s->raw.bFrameEnd;
	return 0;
}

void hi_venc_hal_release_stream(struct hi_venc_chan *c, struct hi_venc_stream *s)
{
	HI_DRV_VENC_ReleaseStream(c->hVenc, &s->raw);
}
