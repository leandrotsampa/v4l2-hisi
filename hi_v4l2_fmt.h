/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V4L2 driver for Hisilicon (HiSTBLinux) video codec hardware.
 * Format mapping: V4L2 fourcc <-> Hisilicon HI_UNF_VCODEC_TYPE_E.
 *
 * Primary target SoC: Hi3798Cv200.
 *
 * Dynamic capability discovery: HI_DRV_VDEC_GetCap() exists and is exported,
 * but has no public prototype and returns a VDEC_CAP_S defined only in the
 * internal vfmw.h. To keep this module self-contained it is wired behind
 * HI_V4L2_USE_VDEC_GETCAP (off by default); when off we advertise the full
 * real codec table (every entry maps to a real HI_UNF_VCODEC_TYPE_E). Turn it
 * on once the prototype/header are confirmed against the SDK build tree.
 */
#ifndef __HI_V4L2_FMT_H__
#define __HI_V4L2_FMT_H__

#include <linux/videodev2.h>
#include "hi_drv_vdec.h"   /* HI_UNF_VCODEC_TYPE_E, HI_DRV_PIX_FORMAT_E, ... */

/*
 * Coded fourccs absent from the 4.4 UAPI but supported by the hardware.
 * Values match upstream so userspace built against newer headers interops.
 */
#ifndef V4L2_PIX_FMT_HEVC
#define V4L2_PIX_FMT_HEVC  v4l2_fourcc('H', 'E', 'V', 'C')
#endif
#ifndef V4L2_PIX_FMT_VP9
#define V4L2_PIX_FMT_VP9   v4l2_fourcc('V', 'P', '9', '0')
#endif
#ifndef V4L2_PIX_FMT_VC1_ANNEX_G
#define V4L2_PIX_FMT_VC1_ANNEX_G  v4l2_fourcc('V', 'C', '1', 'G')
#endif

#define HI_V4L2_CODED  (1 << 0)  /* coded format the decoder accepts (OUTPUT)   */
#define HI_V4L2_RAW    (1 << 1)  /* raw format (decoder CAPTURE / encoder OUTPUT) */
#define HI_V4L2_ENC    (1 << 2)  /* coded format the encoder produces (CAPTURE) */

struct hi_v4l2_fmt {
	u32			fourcc;     /* V4L2_PIX_FMT_*                      */
	HI_UNF_VCODEC_TYPE_E	vcodec;     /* HI_UNF_VCODEC_TYPE_* (coded only)   */
	HI_DRV_PIX_FORMAT_E	pixfmt;     /* HI_DRV_PIX_FMT_* (raw only)         */
	u8			num_planes; /* contiguous planes                   */
	u8			depth;      /* bits per pixel (raw only)           */
	u32			flags;      /* HI_V4L2_CODED / HI_V4L2_RAW         */
};

const struct hi_v4l2_fmt *hi_v4l2_find_fmt(u32 fourcc, u32 flag);
const struct hi_v4l2_fmt *hi_v4l2_enum_fmt(unsigned int idx, u32 flag);

/*
 * Narrow the advertised coded list to what this SoC accelerates. No-op unless
 * HI_V4L2_USE_VDEC_GETCAP is defined. Returns the number of coded formats kept
 * (or -1 when the feature is compiled out).
 */
int hi_v4l2_probe_hw_caps(void);

/*
 * Narrow the advertised encode list to what the VENC actually accepts. @can_encode
 * is called per encode-candidate format (a try-create against the VENC); the entry
 * is kept iff it returns true. Run once at encoder probe. Returns the number kept.
 */
int hi_v4l2_probe_enc_caps(bool (*can_encode)(const struct hi_v4l2_fmt *fmt));

/* CAPTURE sizeimage for a decoded frame of the given raw fourcc. */
u32 hi_v4l2_raw_sizeimage(u32 fourcc, u32 width, u32 height, u32 *bytesperline);

/* Fill a stepwise VIDIOC_ENUM_FRAMESIZES reply (16-aligned, [16..max]). */
int hi_v4l2_fill_framesize(struct v4l2_frmsizeenum *fs, u32 maxw, u32 maxh);

#endif /* __HI_V4L2_FMT_H__ */
