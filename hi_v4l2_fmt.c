// SPDX-License-Identifier: GPL-2.0
/*
 * Format mapping table for the Hisilicon V4L2 codec driver.
 *
 * The coded list mirrors HI_UNF_VCODEC_TYPE_E (source/msp/include/hi_unf_video.h).
 * Optional runtime narrowing via HI_DRV_VDEC_GetCap() is guarded by
 * HI_V4L2_USE_VDEC_GETCAP (see hi_v4l2_fmt.h for the rationale).
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include "hi_v4l2_fmt.h"

#ifdef HI_V4L2_USE_VDEC_GETCAP
#include "vfmw.h"                               /* VDEC_CAP_S, VID_STD_E */
extern HI_S32 HI_DRV_VDEC_GetCap(VDEC_CAP_S *pstCap);  /* exported, no header */
#endif

static const struct hi_v4l2_fmt formats[] = {
	/* ---- coded (OUTPUT queue) ---- */
	{ V4L2_PIX_FMT_MPEG2, HI_UNF_VCODEC_TYPE_MPEG2, HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_MPEG4, HI_UNF_VCODEC_TYPE_MPEG4, HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_XVID,  HI_UNF_VCODEC_TYPE_MPEG4, HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_H263,  HI_UNF_VCODEC_TYPE_H263,  HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	/* Encode candidates carry HI_V4L2_ENC; which ones the HW *actually* encodes
	 * is decided at runtime by hi_v4l2_probe_enc_caps() (a try-create against the
	 * VENC), not hardcoded — on this SoC the VEDU rejects HEVC ("NOT support
	 * Type 36") and the JPGE block may be absent (pJpgeFunc NULL), so the probe
	 * typically keeps only H264, but a build/HW that adds them is picked up. */
	{ V4L2_PIX_FMT_H264,  HI_UNF_VCODEC_TYPE_H264,  HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED | HI_V4L2_ENC },
	{ V4L2_PIX_FMT_HEVC,  HI_UNF_VCODEC_TYPE_HEVC,  HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED | HI_V4L2_ENC },
	/* VC-1 (Advanced Profile) + VP9: decode validado em HW após os fixes de
	 * Dynamic Frame Store (ADR-0010) e VC-1 AdvProfile (KI-016 RESOLVIDO). */
	{ V4L2_PIX_FMT_VC1_ANNEX_G, HI_UNF_VCODEC_TYPE_VC1, HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_VP8,   HI_UNF_VCODEC_TYPE_VP8,   HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_VP9,   HI_UNF_VCODEC_TYPE_VP9,   HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED },
	{ V4L2_PIX_FMT_MJPEG, HI_UNF_VCODEC_TYPE_MJPEG, HI_DRV_PIX_BUTT, 1, 0, HI_V4L2_CODED | HI_V4L2_ENC },

	/* ---- decoded (decoder CAPTURE / encoder OUTPUT) ---- */
	{ V4L2_PIX_FMT_NV12,  HI_UNF_VCODEC_TYPE_BUTT, HI_DRV_PIX_FMT_NV12, 1, 12, HI_V4L2_RAW },
	{ V4L2_PIX_FMT_NV21,  HI_UNF_VCODEC_TYPE_BUTT, HI_DRV_PIX_FMT_NV21, 1, 12, HI_V4L2_RAW },
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* default-on; narrowed per-entry by the runtime cap probes:
 *  - coded_enabled: decode, by hi_v4l2_probe_hw_caps() (VDEC GetCap)
 *  - enc_enabled:   encode, by hi_v4l2_probe_enc_caps() (VENC try-create)
 * Neither is hardcoded — both reflect what the HW actually does. */
static bool coded_enabled[NUM_FORMATS] = { [0 ... NUM_FORMATS - 1] = true };
static bool enc_enabled[NUM_FORMATS]   = { [0 ... NUM_FORMATS - 1] = true };

/* Enablement is per use (flag): a codec may decode but not encode (e.g. HEVC). */
static bool fmt_is_enabled(unsigned int i, u32 flag)
{
	if (flag & HI_V4L2_ENC)
		return enc_enabled[i];
	if (formats[i].flags & HI_V4L2_CODED)
		return coded_enabled[i];
	return true;
}

const struct hi_v4l2_fmt *hi_v4l2_find_fmt(u32 fourcc, u32 flag)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++)
		if (formats[i].fourcc == fourcc &&
		    (formats[i].flags & flag) && fmt_is_enabled(i, flag))
			return &formats[i];
	return NULL;
}

const struct hi_v4l2_fmt *hi_v4l2_enum_fmt(unsigned int idx, u32 flag)
{
	unsigned int i, n = 0;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (!(formats[i].flags & flag) || !fmt_is_enabled(i, flag))
			continue;
		if (n == idx)
			return &formats[i];
		n++;
	}
	return NULL;
}

/*
 * Encode capability probe: ask the VENC (via the caller's try-create callback)
 * which encode candidates it actually accepts, instead of hardcoding. Run once
 * at encoder probe. Returns the number of encode formats kept.
 */
int hi_v4l2_probe_enc_caps(bool (*can_encode)(const struct hi_v4l2_fmt *))
{
	unsigned int i;
	int kept = 0;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (!(formats[i].flags & HI_V4L2_ENC))
			continue;
		enc_enabled[i] = can_encode(&formats[i]);
		if (enc_enabled[i])
			kept++;
	}
	return kept;
}

#ifdef HI_V4L2_USE_VDEC_GETCAP
static int vcodec_to_vidstd(HI_UNF_VCODEC_TYPE_E t)
{
	switch (t) {
	case HI_UNF_VCODEC_TYPE_MPEG2: return VFMW_MPEG2;
	case HI_UNF_VCODEC_TYPE_MPEG4: return VFMW_MPEG4;
	case HI_UNF_VCODEC_TYPE_H263:  return VFMW_H263;
	case HI_UNF_VCODEC_TYPE_H264:  return VFMW_H264;
	case HI_UNF_VCODEC_TYPE_HEVC:  return VFMW_HEVC;
	case HI_UNF_VCODEC_TYPE_VC1:   return VFMW_VC1;
	case HI_UNF_VCODEC_TYPE_VP8:   return VFMW_VP8;
	case HI_UNF_VCODEC_TYPE_VP9:   return VFMW_VP9;
	case HI_UNF_VCODEC_TYPE_MJPEG: return VFMW_JPEG;
	default:                       return -1;   /* no VFMW std equivalent */
	}
}

int hi_v4l2_probe_hw_caps(void)
{
	VDEC_CAP_S cap;
	unsigned int i, j;
	int kept = 0;

	if (HI_DRV_VDEC_GetCap(&cap) != HI_SUCCESS)
		return NUM_FORMATS;        /* keep full table on failure */

	pr_info("hi-v4l2: VDEC HW cap: maxChan=%d maxW=%d maxH=%d maxBitRate=%d\n",
		cap.s32MaxChanNum, cap.s32MaxFrameWidth, cap.s32MaxFrameHeight,
		cap.s32MaxBitRate);
	for (j = 0; j < ARRAY_SIZE(cap.SupportedStd); j++)
		pr_info("hi-v4l2:   SupportedStd[%u] = %d\n", j,
			(int)cap.SupportedStd[j]);

	for (i = 0; i < NUM_FORMATS; i++) {
		int std;

		if (!(formats[i].flags & HI_V4L2_CODED))
			continue;
		std = vcodec_to_vidstd(formats[i].vcodec);
		coded_enabled[i] = false;
		if (std < 0)
			continue;
		for (j = 0; j < ARRAY_SIZE(cap.SupportedStd); j++) {
			/* the firmware terminates the list with VFMW_END_RESERVED;
			 * trailing entries are zero-padding (== VFMW_H264), so stop
			 * here to avoid spurious matches. */
			if ((int)cap.SupportedStd[j] == VFMW_END_RESERVED)
				break;
			if ((int)cap.SupportedStd[j] == std) {
				coded_enabled[i] = true;
				kept++;
				break;
			}
		}
	}
	return kept;
}
#else
int hi_v4l2_probe_hw_caps(void)
{
	return -1;                         /* feature compiled out */
}
#endif /* HI_V4L2_USE_VDEC_GETCAP */

u32 hi_v4l2_raw_sizeimage(u32 fourcc, u32 width, u32 height, u32 *bytesperline)
{
	const struct hi_v4l2_fmt *f = hi_v4l2_find_fmt(fourcc, HI_V4L2_RAW);
	u32 bpl;

	if (!f)
		f = hi_v4l2_find_fmt(V4L2_PIX_FMT_NV12, HI_V4L2_RAW);

	bpl = ALIGN(width, 16);              /* luma stride, 16-byte aligned */
	if (bytesperline)
		*bytesperline = bpl;

	/* NV12/NV21: Y plane + interleaved CbCr at half height => 3/2 * Y. */
	return bpl * ALIGN(height, 16) * f->depth / 8;
}

int hi_v4l2_fill_framesize(struct v4l2_frmsizeenum *fs, u32 maxw, u32 maxh)
{
	if (fs->index)
		return -EINVAL;
	if (!hi_v4l2_find_fmt(fs->pixel_format, HI_V4L2_CODED) &&
	    !hi_v4l2_find_fmt(fs->pixel_format, HI_V4L2_RAW) &&
	    !hi_v4l2_find_fmt(fs->pixel_format, HI_V4L2_ENC))
		return -EINVAL;

	/*
	 * Step must be 2 (not 16): the advertised step constrains which
	 * resolutions a caps negotiator (GStreamer v4l2videodec) will accept.
	 * Common video heights are NOT multiples of 16 (360, 1080, 540) — a
	 * step of 16 makes gst intersect the stream caps to EMPTY and the
	 * pipeline fails with not-negotiated. 4:2:0 only requires even dims;
	 * the HW aligns to 16 internally (buffer/sizeimage), independent of
	 * the dimensions reported here.
	 */
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width   = 16;
	fs->stepwise.max_width   = maxw;
	fs->stepwise.step_width  = 2;
	fs->stepwise.min_height  = 16;
	fs->stepwise.max_height  = maxh;
	fs->stepwise.step_height = 2;
	return 0;
}
