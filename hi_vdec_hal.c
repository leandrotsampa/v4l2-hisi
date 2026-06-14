// SPDX-License-Identifier: GPL-2.0
/*
 * HAL over HI_DRV_VDEC_* (vdec_v2.0). See hi_vdec_hal.h for rationale.
 *
 * Lifecycle mirrors the MPI sequence used by AVPLAY (hi_mpi_avplay.c) and the
 * exported DRV API:
 *   Open -> AllocChan -> SetChanAttr -> ChanBufferInit(INVALID=RAM)
 *        -> CreatePort(STR)+SetPortType(VIRTUAL)+EnablePort -> ChanStart
 *   loop: GetEsBuf/memcpy/PutEsBuf  ||  RecvFrmBuf -> ... -> RlsFrmBuf
 *   ChanStop -> DestroyPort -> ChanBufferDeInit -> FreeChan
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include "hi_vdec_hal.h"
#include "hi_drv_win.h"		/* VO virtual window: HI_DRV_WIN_* (vdp_v4_0) */
#include "drv_win_ioctl.h"	/* WIN_CREATE_S / CMD_WIN_CREATE (DRV_WIN_Process) */
#include "hi_drv_vpss.h"	/* HI_DRV_VPSS_{Get,Set}PortCfg / HI_DRV_VPSS_PORT_CFG_S */
#include "hi_drv_disp.h"	/* HI_DRV_DISP_*Cast* + HI_DRV_DISP_CAST_CFG_S (WBC) */

/*
 * The handle returned by HI_DRV_VDEC_CreatePort IS the VPSS port handle
 * (drv_vdec_intf_k.c:4864 pfnVpssCreatePort -> phPort), so it can be passed
 * straight to the EXPORT_SYMBOL'd VPSS port-cfg helpers. We use this to force
 * the VPSS to actually run its de-tile/de-compress node: VDEC creates the HD
 * port at output 1920x1080, and for a 1080p stream the decoded frame is also
 * 1920x1080 -> CheckPassThroughForVO() takes the bypass and RecvVpssFrmBuf
 * returns the raw tiled+compressed VFMW frame. Setting an output size that
 * differs from the input defeats the bypass -> the HAL programs nxt2_dcmp_en +
 * tile_format from the input enFormat -> linear NV21 out (HW-VALIDATE).
 * Prototypes come from hi_drv_vpss.h; the symbols are EXPORT_SYMBOL'd. */

/*
 * Exported by vdec_v2.0 (Module.symvers) but missing from the public header
 * hi_drv_vdec.h, so declare them here to avoid implicit declarations.
 *
 * ChanBufferInit/DeInit are the vendor's canonical stream-buffer helpers:
 * they mask the channel handle (& 0xff) and, for the RAM-injection case
 * (hDmxVidChn == HI_INVALID_HANDLE), create+attach (Init) and destroy+detach
 * (DeInit) the buffer. The lower-level HI_DRV_VDEC_CreateStrmBuf forwards the
 * handle UNMASKED to the internal attach, which indexes astChanEntity[] out
 * of bounds and faults (KI-011) — so always go through ChanBufferInit.
 */
extern HI_S32 HI_DRV_VDEC_ChanBufferInit(HI_HANDLE hHandle, HI_U32 u32BufSize,
					 HI_HANDLE hDmxVidChn);
extern HI_S32 HI_DRV_VDEC_ChanBufferDeInit(HI_HANDLE hHandle);

/* CMA/dma device exposed by hi_v4l2_dec.c (used for cache sync on VPSS frames) */
extern struct device *hi_dec_dma_dev(void);

struct hi_vdec_chan {
	HI_HANDLE	hVdec;
	HI_HANDLE	hPort;
	HI_HANDLE	hWin;		/* VO virtual window (win path); 0 = none */
	HI_HANDLE	hCast;		/* display write-back (WBC); 0 = none */
	bool		vpss_recv;	/* vp path: RecvVpssFrmBuf, no window */
	const struct hi_v4l2_fmt *coded;
	u32		es_buf_size;
	bool		started;
};

/*
 * VO window entry points (vdp_v4_0/drv_win.c). The HI_DRV_WIN_* wrappers and the
 * lower-level WIN_AcquireFrame/WIN_ReleaseFrame are all EXPORT_SYMBOL'd; declare
 * them here since drv_window.h is private to the VO module. SetSource attaches
 * the VPSS master port as the window's frame source (enSrcMode = HI_ID_VDEC,
 * hSrc = port handle), exactly as HI_MPI_WIN_SetSource does for AVPLAY
 * (mpi_avplay.c:3746).
 */
extern HI_S32 HI_DRV_WIN_Create(HI_DRV_WIN_ATTR_S *pWinAttr, HI_HANDLE *phWindow);
extern HI_S32 HI_DRV_WIN_Destroy(HI_HANDLE hWindow);
extern HI_S32 HI_DRV_WIN_SetSource(HI_HANDLE hWindow,
				   HI_DRV_WIN_SRC_HANDLE_S *pstSrc);
extern HI_S32 HI_DRV_WIN_SetEnable(HI_HANDLE hWindow, HI_BOOL bEnable);
extern HI_S32 WIN_AcquireFrame(HI_HANDLE hWin, HI_DRV_VIDEO_FRAME_S *pFrameinfo);
extern HI_S32 WIN_ReleaseFrame(HI_HANDLE hWin, HI_DRV_VIDEO_FRAME_S *pFrameinfo);
/* VPSS-port recv/release (also declared in the zc section below; used by the
 * vp variant-B path which has no VO window). */
extern HI_S32 HI_DRV_VDEC_Chan_RecvVpssFrmBuf(HI_HANDLE hVdec,
		HI_DRV_VIDEO_FRAME_PACKAGE_S *pstFrm);
extern HI_S32 HI_DRV_VDEC_UserBuf_Release(HI_HANDLE hVdec,
		HI_DRV_VIDEO_FRAME_S *pstFrm);
/*
 * The exported HI_DRV_WIN_Create leaves WIN_CREATE_S.bVirtScreen UNINITIALIZED
 * (stack garbage) and forces bMCE=TRUE — fine for the ioctl path (the MPI fills
 * the struct) but wrong for a direct kernel caller. HI_MPI_WIN_Create sets
 * bVirtScreen=HI_TRUE, bMCE=HI_FALSE (mpi_win.c:413-414); replicate that exactly
 * by driving DRV_WIN_Process(CMD_WIN_CREATE) ourselves.
 */
extern HI_S32 DRV_WIN_Process(HI_U32 cmd, HI_VOID *arg);

/*
 * De-tile prerequisites, set globally on the vendor VDEC (drv_vdec_intf_k.c):
 *   g_bMapFrmEnable = TRUE  -> the decoder gives a kernel mapping of the frame
 *                             (so the SW un-tile can read it).
 *   MaskCtrlWord |= 0x4     -> "maskcmp": the decoder emits the frame as plain
 *                             NV21_TILE (no VCMP), so the un-tile sees raw tiled
 *                             bytes (VCMP'd bytes would be garbage).
 * Replaces the manual `echo maskcmp/map_frm on > /proc/hisi/msp/vdec_ctrl`.
 * NOTE: both are GLOBAL to all VDEC channels (acceptable for a decode box;
 * per-channel scoping would need a vendor change). Call before ChanStart.
 */
extern HI_U32 MaskCtrlWord;
extern HI_BOOL g_bMapFrmEnable;

void hi_vdec_hal_enable_detile(void)
{
	g_bMapFrmEnable = HI_TRUE;
	MaskCtrlWord |= 0x4;
}

static DEFINE_MUTEX(g_lock);
static int g_open_refcnt;
static int g_win_refcnt;	/* VO/WIN subsystem (win path only) */

/*
 * The VO window subsystem must be brought up before HI_DRV_WIN_Create
 * (WIN_Create errors "WIN is not inited" otherwise) — the kernel analog of
 * libhicodec's HI_UNF_VO_Init. HI_DRV_WIN_Init is itself refcounted, but we
 * gate it behind our own count so it runs once for the win path and is torn
 * down with the last window. No HI_UNF_DISP_Init is needed: a virtual window
 * grabs frames without a physical display (libhicodec's decode path does the
 * same).
 */
static int hi_win_subsys_get(void)
{
	int ret = 0;

	mutex_lock(&g_lock);
	if (g_win_refcnt == 0) {
		ret = HI_DRV_WIN_Init();
		if (ret != HI_SUCCESS) {
			pr_err("hi-v4l2 win: HI_DRV_WIN_Init failed (0x%x)\n", ret);
			ret = -EIO;
			goto out;
		}
	}
	g_win_refcnt++;
out:
	mutex_unlock(&g_lock);
	return ret;
}

static void hi_win_subsys_put(void)
{
	mutex_lock(&g_lock);
	if (--g_win_refcnt == 0)
		HI_DRV_WIN_DeInit();
	if (g_win_refcnt < 0)
		g_win_refcnt = 0;
	mutex_unlock(&g_lock);
}

int hi_vdec_hal_global_init(void)
{
	int ret = 0;

	mutex_lock(&g_lock);
	if (g_open_refcnt == 0) {
		ret = HI_DRV_VDEC_Open();
		if (ret != HI_SUCCESS) {
			pr_err("hi-v4l2: HI_DRV_VDEC_Open failed (0x%x)\n", ret);
			ret = -EIO;
			goto out;
		}
	}
	g_open_refcnt++;
out:
	mutex_unlock(&g_lock);
	return ret;
}

void hi_vdec_hal_global_exit(void)
{
	mutex_lock(&g_lock);
	if (--g_open_refcnt == 0)
		HI_DRV_VDEC_Close();
	if (g_open_refcnt < 0)
		g_open_refcnt = 0;
	mutex_unlock(&g_lock);
}

static HI_UNF_VCODEC_CAP_LEVEL_E cap_level_for(u32 w, u32 h)
{
	u32 m = max(w, h);

	/*
	 * The cap level sizes the VFMW VDH working buffer. Requesting 4K for HD
	 * content over-allocates (a 4K frame store x refs > the ~50 MB free in
	 * the 60 MB MMZ pool), so ChanStart fails with "alloc VDH MMZ err"
	 * (KI-013). Grade it to the actual resolution, mirroring the encoder HAL.
	 */
	if (m > 2160)
		return HI_UNF_VCODEC_CAP_LEVEL_4096x2160;
	if (m > 1280)
		return HI_UNF_VCODEC_CAP_LEVEL_FULLHD;
	if (m > 720)
		return HI_UNF_VCODEC_CAP_LEVEL_720P;
	return HI_UNF_VCODEC_CAP_LEVEL_D1;
}

static HI_UNF_VCODEC_PRTCL_LEVEL_E prtcl_level_for(HI_UNF_VCODEC_TYPE_E t)
{
	switch (t) {
	case HI_UNF_VCODEC_TYPE_H264:
	case HI_UNF_VCODEC_TYPE_HEVC:
	case HI_UNF_VCODEC_TYPE_VP9:
	case HI_UNF_VCODEC_TYPE_VC1:
		return HI_UNF_VCODEC_PRTCL_LEVEL_H264;
	default:
		return HI_UNF_VCODEC_PRTCL_LEVEL_MPEG;
	}
}

struct hi_vdec_chan *hi_vdec_hal_create(const struct hi_v4l2_fmt *coded,
					u32 width, u32 height, u32 es_buf_size)
{
	struct hi_vdec_chan *c;
	HI_UNF_AVPLAY_OPEN_OPT_S opt = {0};
	HI_UNF_VCODEC_ATTR_S attr = {0};
	int ret;

	if (!coded || !(coded->flags & HI_V4L2_CODED))
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->coded = coded;
	c->es_buf_size = es_buf_size;

	opt.enDecType       = HI_UNF_VCODEC_DEC_TYPE_NORMAL;
	opt.enCapLevel      = cap_level_for(width, height);
	opt.enProtocolLevel = prtcl_level_for(coded->vcodec);

	ret = HI_DRV_VDEC_AllocChan(&c->hVdec, &opt);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: AllocChan failed (0x%x)\n", ret);
		goto err_free;
	}

	attr.enType       = coded->vcodec;
	attr.enMode       = HI_UNF_VCODEC_MODE_NORMAL;
	attr.u32ErrCover  = 100;
	attr.u32Priority  = 3;
	attr.bOrderOutput = HI_TRUE;
	/*
	 * VC-1: our fourcc is V4L2_PIX_FMT_VC1_ANNEX_G (= Advanced Profile). The
	 * vendor needs unExtAttr.stVC1Attr.bAdvancedProfile set, otherwise the VFMW
	 * parses the stream as Simple/Main profile and rejects it ("Vc1SMPSeqHdr:
	 * picture width out of range" / "VC1 S/MP BS is wrong"). CodecVersion mirrors
	 * the AVPLAY/esplay default. See KI-016.
	 */
	if (coded->vcodec == HI_UNF_VCODEC_TYPE_VC1) {
		attr.unExtAttr.stVC1Attr.bAdvancedProfile = HI_TRUE;
		attr.unExtAttr.stVC1Attr.u32CodecVersion  = 8;
	}
	ret = HI_DRV_VDEC_SetChanAttr(c->hVdec, &attr);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: SetChanAttr failed (0x%x)\n", ret);
		goto err_freechan;
	}

	/*
	 * RAM ES injection (no demux): create + attach the stream buffer in one
	 * step via the vendor helper. hDmxVidChn = HI_INVALID_HANDLE selects the
	 * RAM path; ChanBufferInit masks the handle and also attaches, so we must
	 * NOT call CreateStrmBuf/Chan_AttachStrmBuf directly (see KI-011).
	 */
	ret = HI_DRV_VDEC_ChanBufferInit(c->hVdec, es_buf_size, HI_INVALID_HANDLE);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: ChanBufferInit failed (0x%x)\n", ret);
		goto err_freechan;
	}

	/* Single virtual output port: frames are pulled with RecvFrmBuf. */
	ret = HI_DRV_VDEC_CreatePort(c->hVdec, &c->hPort, VDEC_PORT_STR);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2: CreatePort failed (0x%x)\n", ret);
		goto err_bufdeinit;
	}
	HI_DRV_VDEC_SetPortType(c->hVdec, c->hPort, VDEC_PORT_TYPE_VIRTUAL);
	HI_DRV_VDEC_EnablePort(c->hVdec, c->hPort);

	return c;

err_bufdeinit:
	HI_DRV_VDEC_ChanBufferDeInit(c->hVdec);
err_freechan:
	HI_DRV_VDEC_FreeChan(c->hVdec);
err_free:
	kfree(c);
	return ERR_PTR(-EIO);
}

void hi_vdec_hal_destroy(struct hi_vdec_chan *c)
{
	if (!c)
		return;
	if (c->started)
		hi_vdec_hal_stop(c);
	if (c->hCast) {
		HI_DRV_DISP_SetCastEnable(c->hCast, HI_FALSE);
		HI_DRV_DISP_DestroyCast(c->hCast);
		c->hCast = 0;
	}
	if (c->hWin) {
		HI_DRV_WIN_SRC_HANDLE_S src = {0};

		/* Detach (BUTT/INVALID) before destroy, mirroring AVPLAY's
		 * HI_UNF_VO_DetachWindow -> SetSource(HI_ID_BUTT). */
		HI_DRV_WIN_SetEnable(c->hWin, HI_FALSE);
		src.enSrcMode = HI_ID_BUTT;
		src.hSrc = HI_INVALID_HANDLE;
		src.hSecondSrc = HI_INVALID_HANDLE;
		HI_DRV_WIN_SetSource(c->hWin, &src);
		HI_DRV_WIN_Destroy(c->hWin);
		c->hWin = 0;
		hi_win_subsys_put();
	}
	if (c->hPort)
		HI_DRV_VDEC_DestroyPort(c->hVdec, c->hPort);
	/* DeInit destroys the BUFMNG stream buffer AND detaches it (avoids the
	 * leak left by a bare Chan_DetachStrmBuf). Masks the handle internally. */
	HI_DRV_VDEC_ChanBufferDeInit(c->hVdec);
	HI_DRV_VDEC_FreeChan(c->hVdec);
	kfree(c);
}

int hi_vdec_hal_start(struct hi_vdec_chan *c)
{
	int ret;

	if (c->started)
		return 0;
	ret = HI_DRV_VDEC_ChanStart(c->hVdec);
	if (ret != HI_SUCCESS)
		return -EIO;
	c->started = true;
	return 0;
}

int hi_vdec_hal_stop(struct hi_vdec_chan *c)
{
	if (!c->started)
		return 0;
	HI_DRV_VDEC_ChanStop(c->hVdec);
	c->started = false;
	return 0;
}

int hi_vdec_hal_feed_es(struct hi_vdec_chan *c, const void *data, u32 len,
			u64 pts, bool eof)
{
	VDEC_ES_BUF_S es = {0};
	int ret;

	if (!len)
		return 0;

	/*
	 * Request a write slot of @len bytes. GetEsBuf forwards u32BufSize to
	 * BUFMNG_GetWriteBuffer as the reservation size; passing 0 here makes
	 * BUFMNG hand back a 0-byte slot, so the clamp below would zero @len and
	 * we would feed an empty AU. The VFMW then keeps calling read_stream but
	 * VDEC_GetEsFromBM finds nothing (ret=0xffffffff) and never schedules a
	 * decode task. Confirmed on HW via the HI-V4L2-T4 read_stream probe.
	 */
	es.u32BufSize = len;
	ret = HI_DRV_VDEC_GetEsBuf(c->hVdec, &es);
	if (ret != HI_SUCCESS || !es.pu8Addr)
		return -EAGAIN;                 /* ES ring full; retry later */

	if (es.u32BufSize < len)
		len = es.u32BufSize;            /* caller must re-feed remainder */

	memcpy(es.pu8Addr, data, len);
	es.u32BufSize    = len;
	es.u64Pts        = pts;
	es.bEndOfFrame   = eof ? HI_TRUE : HI_FALSE;
	es.bDiscontinuous = HI_FALSE;

	ret = HI_DRV_VDEC_PutEsBuf(c->hVdec, &es);
	if (ret != HI_SUCCESS)
		return -EIO;

	return len;
}

int hi_vdec_hal_set_eos(struct hi_vdec_chan *c)
{
	return (HI_DRV_VDEC_SetEosFlag(c->hVdec) == HI_SUCCESS) ? 0 : -EIO;
}

/*
 * Map a VPSS-produced LINEAR NV12 frame's planes for the memcpy drain. The
 * buffer lives in reserved MMZ (outside the kernel linear map); the SDK reads
 * it through an UNCACHED mapping (userspace HI_MMZ_Map(..., HI_FALSE)). We do
 * the same with ioremap so copy_frame_to_buf's memcpy sees the HW writes
 * directly — avoiding the cached-stale trap (RE notes 2026-06-13). NV12/NV21 is
 * semi-planar contiguous, so one mapping spans both planes. Prefers any kernel
 * vaddr the producer already attached. Returns 0 on success.
 */
static int hi_vdec_map_frame(struct hi_vdec_frame *f)
{
	HI_DRV_VIDEO_FRAME_S *fr = &f->raw;
	u32 ysize, csize, coff;

	if (fr->stBufAddr[0].vir_addr_y) {
		f->virt_y = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_y;
		f->virt_c = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_c;
		f->map_base = NULL;
		return 0;
	}

	ysize = f->stride_y * f->height;
	csize = f->stride_c * (f->height / 2);
	coff  = (f->phys_c > f->phys_y) ? (f->phys_c - f->phys_y) : ysize;
	f->map_size = coff + csize;
	/*
	 * The VPSS output buffer is in reserved System RAM (MMZ/CMA, in the kernel
	 * memory map) — ioremap REFUSES it (arch/arm/mm/ioremap.c:301 WARN, returns
	 * NULL). memremap(MEMREMAP_WB) is the correct API for System RAM and returns
	 * the existing kernel linear mapping.
	 */
	f->map_base = memremap(f->phys_y, f->map_size, MEMREMAP_WB);
	if (!f->map_base)
		return -ENOMEM;
	/*
	 * memremap(WB) is a CACHED view; the VPSS wrote the frame via DMA, so the
	 * CPU cache for this range is stale (reads come back blocky/garbage).
	 * Invalidate it for the CPU before the memcpy drain. No CPU-side IOMMU
	 * here, so phys == dma addr.
	 */
	dma_sync_single_for_cpu(hi_dec_dma_dev(), (dma_addr_t)f->phys_y,
				f->map_size, DMA_FROM_DEVICE);
	f->virt_y = f->map_base;
	f->virt_c = (u8 *)f->map_base + coff;
	return 0;
}

static void hi_vdec_unmap_frame(struct hi_vdec_frame *f)
{
	if (f->map_base) {
		memunmap(f->map_base);
		f->map_base = NULL;
	}
}

static void hi_vdec_fill_frame(struct hi_vdec_frame *f)
{
	HI_DRV_VIDEO_FRAME_S *fr = &f->raw;

	f->width    = fr->u32Width;
	f->height   = fr->u32Height;
	f->stride_y = fr->stBufAddr[0].u32Stride_Y;
	f->stride_c = fr->stBufAddr[0].u32Stride_C;
	f->phys_y   = fr->stBufAddr[0].u32PhyAddr_Y;
	f->phys_c   = fr->stBufAddr[0].u32PhyAddr_C;
	f->pts      = fr->u32Pts;
	f->last     = (f->phys_y == 0);
}

/*
 * win path release (MCE-style): the frame came from RecvVpssFrmBuf (vpss_recv).
 * After the memcpy drain, hand it to the window with HI_DRV_WIN_QFrame instead of
 * releasing it back to the VPSS pool directly — this drives the VPSS to keep
 * producing content and recycles the buffer through the window/VO (the bare
 * recv->VORlsFrame cycle yielded empty frames). recv stays vp_recv (vpss_recv).
 */
static void hi_vdec_hal_win_release(struct hi_vdec_chan *c,
				    struct hi_vdec_frame *f)
{
	if (f->last || !f->phys_y)
		return;
	hi_vdec_unmap_frame(f);
	/* f->raw is the LINEAR NV21 frame from WIN_AcquireFrame; return it to the
	 * virtual window's pool. */
	WIN_ReleaseFrame(c->hWin, &f->raw);
}

/*
 * vp (variant B) recv: VPSS-managed path WITHOUT a VO window. The VPSS allocates
 * and (we are testing whether it) processes its own LINEAR NV12 output; we pull
 * it straight off the VPSS port with RecvVpssFrmBuf — the same recv that already
 * returned frames in the zero-copy probe (ours=1) — then map+memcpy. Avoids the
 * VO-window push/attach handshake and the master-port free-run wedge. Release
 * recycles the VPSS buffer via UserBuf_Release (-> VDEC_Chan_VORlsFrame).
 */
static int hi_vdec_hal_vp_recv(struct hi_vdec_chan *c, struct hi_vdec_frame *f)
{
	HI_DRV_VIDEO_FRAME_PACKAGE_S *pkg;	/* ~2.6 KB: keep off the stack */
	int ret;

	memset(f, 0, sizeof(*f));

	/*
	 * Virtual-window path (bVirtual=TRUE, enDataFormat=NV21): MCE-style pump —
	 * RecvVpssFrmBuf pulls the native (102 tiled+CMP) frame off the VPSS port,
	 * QFrame hands it to the virtual window, which (having no scanout to de-tile
	 * in) converts to its LINEAR NV21 pool; WIN_AcquireFrame returns that linear
	 * frame. (drv_mce_avplay.c:1134 RecvVpssFrmBuf -> :969 HI_DRV_WIN_QFrame.)
	 */
	if (c->hWin) {
		pkg = kzalloc(sizeof(*pkg), GFP_KERNEL);
		if (!pkg)
			return -ENOMEM;
		ret = HI_DRV_VDEC_Chan_RecvVpssFrmBuf(c->hVdec, pkg);
		if (ret == HI_SUCCESS && pkg->u32FrmNum != 0) {
			/* push native frame to the virtual window for NV21 conversion;
			 * the window releases it back to the VPSS via its source. */
			HI_DRV_WIN_QFrame(c->hWin, &pkg->stFrame[0].stFrameVideo);
		}
		kfree(pkg);

		ret = WIN_AcquireFrame(c->hWin, &f->raw);
		if (ret != HI_SUCCESS)
			return -EAGAIN;		/* no converted linear frame yet */
		hi_vdec_fill_frame(f);
		{
			static int wino;
			if (!wino) {
				wino = 1;
				pr_info("hi-v4l2 WINFRM: fmt=%d %ux%u sY=%u sC=%u pY=0x%x pC=0x%x\n",
					f->raw.ePixFormat, f->width, f->height,
					f->stride_y, f->stride_c, f->phys_y, f->phys_c);
			}
		}
		if (f->last)
			return 0;
		if (hi_vdec_map_frame(f) != 0) {
			WIN_ReleaseFrame(c->hWin, &f->raw);
			return -ENOMEM;
		}
		return 0;
	}

	pkg = kzalloc(sizeof(*pkg), GFP_KERNEL);
	if (!pkg)
		return -ENOMEM;

	ret = HI_DRV_VDEC_Chan_RecvVpssFrmBuf(c->hVdec, pkg);
	if (ret != HI_SUCCESS || pkg->u32FrmNum == 0) {
		kfree(pkg);
		return -EAGAIN;
	}

	/* WIN path: RecvVpssFrmBuf pumped the VPSS detile pipeline; QFrame the frame
	 * to the VO so it DISPLAYS (pstDisplay = de-tiled). CapturePicture then WBCs
	 * the *displayed* (de-tiled, full-res) frame -> our V4L2 CAPTURE. The pumped
	 * frame is owned by the window (released via its VO source callback). */
	if (c->hWin) {
		HI_DRV_WIN_QFrame(c->hWin, &pkg->stFrame[0].stFrameVideo);
		kfree(pkg);
		ret = HI_DRV_WIN_CapturePicture(c->hWin, &f->raw);
		if (ret != HI_SUCCESS)
			return -EAGAIN;
		hi_vdec_fill_frame(f);
		{
			static int capo;
			if (!capo) {
				capo = 1;
				pr_info("hi-v4l2 CAP: fmt=%d %ux%u sY=%u pY=0x%x\n",
					f->raw.ePixFormat, f->width, f->height,
					f->stride_y, f->phys_y);
			}
		}
		if (f->last)
			return 0;
		if (hi_vdec_map_frame(f) != 0) {
			HI_DRV_WIN_CapturePictureRelease(c->hWin, &f->raw);
			return -ENOMEM;
		}
		return 0;
	}

	{	/* HI-V4L2-PKG: dump frame package vs what the VO displays */
		static int dbgc;
		static HI_DRV_WIN_PLAY_INFO_S pi;
		HI_U32 i;
		if (dbgc < 8) {
			dbgc++;
			pr_info("hi-v4l2 PKG: FrmNum=%u\n", pkg->u32FrmNum);
			for (i = 0; i < pkg->u32FrmNum && i < 4; i++)
				pr_info("hi-v4l2 PKG[%u]: fmt=%d pY=0x%x pC=0x%x %ux%u sY=%u\n", i,
					pkg->stFrame[i].stFrameVideo.ePixFormat,
					pkg->stFrame[i].stFrameVideo.stBufAddr[0].u32PhyAddr_Y,
					pkg->stFrame[i].stFrameVideo.stBufAddr[0].u32PhyAddr_C,
					pkg->stFrame[i].stFrameVideo.u32Width,
					pkg->stFrame[i].stFrameVideo.u32Height,
					pkg->stFrame[i].stFrameVideo.stBufAddr[0].u32Stride_Y);
			if (c->hWin &&
			    HI_DRV_WIN_GetPlayInfo(c->hWin, &pi) == HI_SUCCESS)
				pr_info("hi-v4l2 WIN-disp: fmt=%d pY=0x%x %ux%u sY=%u\n",
					pi.newest_playframeinfo.ePixFormat,
					pi.newest_playframeinfo.stBufAddr[0].u32PhyAddr_Y,
					pi.newest_playframeinfo.u32Width,
					pi.newest_playframeinfo.u32Height,
					pi.newest_playframeinfo.stBufAddr[0].u32Stride_Y);
		}
	}

	memcpy(&f->raw, &pkg->stFrame[0].stFrameVideo, sizeof(f->raw));
	kfree(pkg);

	hi_vdec_fill_frame(f);
	{
		static int once;	/* HI-V4L2-VPDBG: one-shot geometry probe */
		if (!once) {
			once = 1;
			pr_info("hi-v4l2 vp: frm fmt=%d %ux%u sY=%u sC=%u pY=0x%x pC=0x%x\n",
				f->raw.ePixFormat, f->width, f->height,
				f->stride_y, f->stride_c, f->phys_y, f->phys_c);
		}
	}
	if (f->last)
		return 0;
	if (hi_vdec_map_frame(f) != 0) {
		HI_DRV_VDEC_UserBuf_Release(c->hVdec, &f->raw);
		return -ENOMEM;
	}
	return 0;
}

static void hi_vdec_hal_vp_release(struct hi_vdec_chan *c,
				   struct hi_vdec_frame *f)
{
	if (f->last || !f->phys_y)
		return;
	hi_vdec_unmap_frame(f);
	HI_DRV_VDEC_UserBuf_Release(c->hVdec, &f->raw);
}

int hi_vdec_hal_recv_frame(struct hi_vdec_chan *c, struct hi_vdec_frame *f)
{
	HI_DRV_VIDEO_FRAME_S *fr = &f->raw;
	int ret;

	/* vpss_recv covers both the vp path and the win path (which also sets it). */
	if (c->vpss_recv)
		return hi_vdec_hal_vp_recv(c, f);

	memset(f, 0, sizeof(*f));
	ret = HI_DRV_VDEC_RecvFrmBuf(c->hVdec, fr);
	if (ret != HI_SUCCESS)
		return -EAGAIN;                 /* no decoded frame ready */

	f->width    = fr->u32Width;
	f->height   = fr->u32Height;
	f->stride_y = fr->stBufAddr[0].u32Stride_Y;
	f->stride_c = fr->stBufAddr[0].u32Stride_C;
	f->phys_y   = fr->stBufAddr[0].u32PhyAddr_Y;
	f->phys_c   = fr->stBufAddr[0].u32PhyAddr_C;
	/* HW-VALIDATE: vir_addr_* is the MMZ kernel mapping on Hi3798Cv200. */
	f->virt_y   = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_y;
	f->virt_c   = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_c;
	f->pts      = fr->u32Pts;
	/*
	 * At end-of-stream the VFMW returns an EOS SENTINEL frame: RecvFrmBuf
	 * succeeds but the frame carries no real buffer (PhyAddr_Y == 0). It must
	 * NOT be copied to a CAPTURE buffer nor released (HI_DRV_VDEC_RlsFrmBuf
	 * rejects PhyAddr=0 -> "VDEC_RlsFrm err Phyaddr=0x0"). Flag it as the last
	 * frame so the caller treats it as EOS instead.
	 */
	f->last     = (fr->stBufAddr[0].u32PhyAddr_Y == 0);
	return 0;
}

void hi_vdec_hal_release_frame(struct hi_vdec_chan *c, struct hi_vdec_frame *f)
{
	/* win path sets BOTH hWin and vpss_recv: QFrame-release takes precedence. */
	if (c->hWin) {
		hi_vdec_hal_win_release(c, f);
		return;
	}
	if (c->vpss_recv) {
		hi_vdec_hal_vp_release(c, f);
		return;
	}
	/* The EOS sentinel (phys_y == 0) owns no buffer; releasing it errors. */
	if (!f->phys_y)
		return;
	HI_DRV_VDEC_RlsFrmBuf(c->hVdec, &f->raw);
}

/*
 * Zero-copy Fase C (KI-003). The VPSS user-managed path: we hand the decoder
 * our own (CMA) frame buffers and it writes the decoded NV12 straight into them
 * — no VFMW->CAPTURE memcpy. Proven on HW 2026-06-12. Used by the production
 * decoder (module param zc=1) and the debugfs probe (hi_v4l2_zc_test.c). See the
 * 2026-06-12 RE notes for the mechanism and the two-path correction.
 */
extern HI_S32 HI_DRV_VDEC_UserBuf_SetMode(HI_HANDLE hVdec, HI_U32 bUserAlloc);
extern HI_S32 HI_DRV_VDEC_UserBuf_Commit(HI_HANDLE hVdec, HI_U32 *pu32PhyAddr,
		HI_U64 *pu64UsrVirAddr, HI_U32 u32BufNum, HI_U32 u32BufSize,
		HI_U32 u32Stride);
extern HI_S32 HI_DRV_VDEC_UserBuf_SetState(HI_HANDLE hVdec, HI_U32 bStart);
extern HI_S32 HI_DRV_VDEC_UserBuf_Release(HI_HANDLE hVdec,
		HI_DRV_VIDEO_FRAME_S *pstFrm);
extern HI_S32 HI_DRV_VDEC_Chan_RecvVpssFrmBuf(HI_HANDLE hVdec,
		HI_DRV_VIDEO_FRAME_PACKAGE_S *pstFrm);

int hi_vdec_hal_zc_commit(struct hi_vdec_chan *c, u32 *phys, u64 *virt,
			  u32 num, u32 size, u32 stride)
{
	int ret;

	ret = HI_DRV_VDEC_UserBuf_SetMode(c->hVdec, 1 /* USER_ALLOC */);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: SetMode(USER_ALLOC) failed (0x%x)\n", ret);
		return -EIO;
	}
	ret = HI_DRV_VDEC_UserBuf_Commit(c->hVdec, (HI_U32 *)phys, (HI_U64 *)virt,
					 num, size, stride);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: Commit(%u bufs size=%u) failed (0x%x)\n",
		       num, size, ret);
		return -EIO;
	}
	ret = HI_DRV_VDEC_UserBuf_SetState(c->hVdec, 1 /* START */);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: SetState(START) failed (0x%x)\n", ret);
		return -EIO;
	}
	pr_info("hi-v4l2 zc: committed %u user buffers (size=%u stride=%u)\n",
		num, size, stride);
	return 0;
}

int hi_vdec_hal_zc_recv(struct hi_vdec_chan *c, struct hi_vdec_frame *f)
{
	HI_DRV_VIDEO_FRAME_PACKAGE_S *pkg;	/* ~2.6 KB: keep off the stack */
	HI_DRV_VIDEO_FRAME_S *fr;
	int ret;

	memset(f, 0, sizeof(*f));

	pkg = kzalloc(sizeof(*pkg), GFP_KERNEL);
	if (!pkg)
		return -ENOMEM;

	ret = HI_DRV_VDEC_Chan_RecvVpssFrmBuf(c->hVdec, pkg);
	if (ret != HI_SUCCESS || pkg->u32FrmNum == 0) {
		kfree(pkg);
		return -EAGAIN;
	}

	fr = &pkg->stFrame[0].stFrameVideo;
	memcpy(&f->raw, fr, sizeof(*fr));
	f->width    = fr->u32Width;
	f->height   = fr->u32Height;
	f->stride_y = fr->stBufAddr[0].u32Stride_Y;
	f->stride_c = fr->stBufAddr[0].u32Stride_C;
	f->phys_y   = fr->stBufAddr[0].u32PhyAddr_Y;
	f->phys_c   = fr->stBufAddr[0].u32PhyAddr_C;
	f->virt_y   = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_y;
	f->virt_c   = (void *)(unsigned long)fr->stBufAddr[0].vir_addr_c;
	f->pts      = fr->u32Pts;
	f->last     = (fr->stBufAddr[0].u32PhyAddr_Y == 0);
	kfree(pkg);
	return 0;
}

void hi_vdec_hal_zc_release(struct hi_vdec_chan *c, struct hi_vdec_frame *f)
{
	if (!f->phys_y)
		return;
	HI_DRV_VDEC_UserBuf_Release(c->hVdec, &f->raw);
}

/*
 * Passo 2b: like hi_vdec_hal_create() but builds a real VPSS pipeline port
 * (VDEC_PORT_HD / MASTER) instead of the VFMW-direct STR virtual port. The STR
 * port bypasses the VPSS (KI-005c), so the user-managed buffers committed to
 * the VPSS never get filled and VpssEventNewFrame fires with NULL frame args.
 * A HD master port routes decoded frames THROUGH the VPSS into our buffers.
 * Mirrors the AVPLAY master-port setup (mpi_avplay.c). Test-only.
 */
struct hi_vdec_chan *hi_vdec_hal_zc_create(const struct hi_v4l2_fmt *coded,
					   u32 width, u32 height, u32 es_buf_size)
{
	struct hi_vdec_chan *c;
	HI_UNF_AVPLAY_OPEN_OPT_S opt = {0};
	HI_UNF_VCODEC_ATTR_S attr = {0};
	int ret;

	if (!coded || !(coded->flags & HI_V4L2_CODED))
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->coded = coded;
	c->es_buf_size = es_buf_size;

	opt.enDecType       = HI_UNF_VCODEC_DEC_TYPE_NORMAL;
	opt.enCapLevel      = cap_level_for(width, height);
	opt.enProtocolLevel = prtcl_level_for(coded->vcodec);

	ret = HI_DRV_VDEC_AllocChan(&c->hVdec, &opt);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: AllocChan failed (0x%x)\n", ret);
		goto err_free;
	}

	attr.enType       = coded->vcodec;
	attr.enMode       = HI_UNF_VCODEC_MODE_NORMAL;
	attr.u32ErrCover  = 100;
	attr.u32Priority  = 3;
	attr.bOrderOutput = HI_TRUE;
	if (coded->vcodec == HI_UNF_VCODEC_TYPE_VC1) {
		attr.unExtAttr.stVC1Attr.bAdvancedProfile = HI_TRUE;
		attr.unExtAttr.stVC1Attr.u32CodecVersion  = 8;
	}
	ret = HI_DRV_VDEC_SetChanAttr(c->hVdec, &attr);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: SetChanAttr failed (0x%x)\n", ret);
		goto err_freechan;
	}

	ret = HI_DRV_VDEC_ChanBufferInit(c->hVdec, es_buf_size, HI_INVALID_HANDLE);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: ChanBufferInit failed (0x%x)\n", ret);
		goto err_freechan;
	}

	/*
	 * CRITICAL ORDER: set USER_ALLOC_MANAGE *before* CreatePort. VDEC_Chan_CreatePort
	 * reads enFrameBuffer at creation time to set the VPSS port's eBufType
	 * (drv_vdec_intf_k.c:4823). If left at the default (VPSS_ALLOC_MANAGE), the VPSS
	 * port is built VPSS-managed and fires NEW_FRAME with NULL args -> our user-managed
	 * handler rejects it ("VpssEventNewFrame args err"). Setting it first makes the VPSS
	 * port user-managed (and auto-sizes output to the stream, :4860 -> no SetResolution).
	 */
	ret = HI_DRV_VDEC_UserBuf_SetMode(c->hVdec, 1 /* USER_ALLOC */);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: SetMode(USER_ALLOC) pre-port failed (0x%x)\n", ret);
		goto err_bufdeinit;
	}

	/* Real VPSS master port (HD) — frames flow through the VPSS, not the
	 * VFMW-direct STR bypass. This port becomes stPort[0], where SetExtBuffer
	 * commits the user buffers. */
	ret = HI_DRV_VDEC_CreatePort(c->hVdec, &c->hPort, VDEC_PORT_HD);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 zc: CreatePort(HD) failed (0x%x)\n", ret);
		goto err_bufdeinit;
	}
	HI_DRV_VDEC_SetPortType(c->hVdec, c->hPort, VDEC_PORT_TYPE_MASTER);
	HI_DRV_VDEC_EnablePort(c->hVdec, c->hPort);

	pr_info("hi-v4l2 zc: VPSS HD master port created (hVdec=0x%x hPort=0x%x)\n",
		c->hVdec, c->hPort);
	return c;

err_bufdeinit:
	HI_DRV_VDEC_ChanBufferDeInit(c->hVdec);
err_freechan:
	HI_DRV_VDEC_FreeChan(c->hVdec);
err_free:
	kfree(c);
	return ERR_PTR(-EIO);
}

/*
 * VO virtual-window decode path. Same VDEC bring-up as hi_vdec_hal_create, but
 * with a real VPSS HD/MASTER port (VPSS-managed buffers) feeding a virtual VO
 * window. The window is the consumer that DRIVES the VPSS to decompress/detile
 * the native frame into LINEAR NV12 — recv_frame/release_frame then pull it via
 * WIN_AcquireFrame/WIN_ReleaseFrame (dispatched on c->hWin). Mirrors libhicodec
 * (HI_UNF_VO_CreateWindow[bVirtual] + AttachWindow + AcquireFrame) and the
 * AVPLAY attach (mpi_avplay.c:3746: enSrcMode=HI_ID_VDEC, hSrc=master port).
 */
struct hi_vdec_chan *hi_vdec_hal_win_create(const struct hi_v4l2_fmt *coded,
					    u32 width, u32 height, u32 es_buf_size)
{
	struct hi_vdec_chan *c;
	HI_UNF_AVPLAY_OPEN_OPT_S opt = {0};
	HI_UNF_VCODEC_ATTR_S attr = {0};
	HI_DRV_WIN_ATTR_S wattr = {0};
	HI_DRV_WIN_SRC_HANDLE_S src = {0};
	WIN_CREATE_S wc = {0};
	VDEC_PORT_PARAM_S pp = {0};	/* MCE DRV_AVPLAY_CreatePort: GetPortParam */
	int ret;

	if (!coded || !(coded->flags & HI_V4L2_CODED))
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->coded = coded;
	c->es_buf_size = es_buf_size;
	/*
	 * MCE-style loop (drv_mce_avplay.c): attach the window (reconfigures the VPSS
	 * port for the de-tile transform), then per frame: RecvVpssFrmBuf (vpss_recv)
	 * -> memcpy to vb2 -> HI_DRV_WIN_QFrame back to the window. The QFrame is what
	 * drives the VPSS to actually write content and recycles the buffer (the bare
	 * recv->release cycle produced empty frames). recv via vpss_recv; release via
	 * win_release (QFrame), so hWin is checked before vpss_recv in release_frame.
	 */
	c->vpss_recv = true;

	opt.enDecType       = HI_UNF_VCODEC_DEC_TYPE_NORMAL;
	opt.enCapLevel      = cap_level_for(width, height);
	opt.enProtocolLevel = prtcl_level_for(coded->vcodec);

	ret = HI_DRV_VDEC_AllocChan(&c->hVdec, &opt);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: AllocChan failed (0x%x)\n", ret);
		goto err_free;
	}

	attr.enType       = coded->vcodec;
	attr.enMode       = HI_UNF_VCODEC_MODE_NORMAL;
	attr.u32ErrCover  = 100;
	attr.u32Priority  = 3;
	attr.bOrderOutput = HI_TRUE;
	if (coded->vcodec == HI_UNF_VCODEC_TYPE_VC1) {
		attr.unExtAttr.stVC1Attr.bAdvancedProfile = HI_TRUE;
		attr.unExtAttr.stVC1Attr.u32CodecVersion  = 8;
	}
	ret = HI_DRV_VDEC_SetChanAttr(c->hVdec, &attr);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: SetChanAttr failed (0x%x)\n", ret);
		goto err_freechan;
	}

	ret = HI_DRV_VDEC_ChanBufferInit(c->hVdec, es_buf_size, HI_INVALID_HANDLE);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: ChanBufferInit failed (0x%x)\n", ret);
		goto err_freechan;
	}

	/* VPSS HD master port, VPSS-managed buffers (NO UserBuf_SetMode): the
	 * VPSS allocates its own linear output and the window consumes it. */
	ret = HI_DRV_VDEC_CreatePort(c->hVdec, &c->hPort, VDEC_PORT_HD);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: CreatePort(HD) failed (0x%x)\n", ret);
		goto err_bufdeinit;
	}
	/* MCE DRV_AVPLAY_CreatePort does GetPortParam right after CreatePort,
	 * before WIN_SetSource — our earlier win path omitted it (0 frames). */
	HI_DRV_VDEC_GetPortParam(c->hVdec, c->hPort, &pp);
	HI_DRV_VDEC_SetPortType(c->hVdec, c->hPort, VDEC_PORT_TYPE_MASTER);
	HI_DRV_VDEC_EnablePort(c->hVdec, c->hPort);	/* required: no frames without it */

	/* Bring up the VO/WIN subsystem before creating the window. */
	ret = hi_win_subsys_get();
	if (ret)
		goto err_destroyport;

	/* VIRTUAL VO window (decode-to-memory, NO display) — the consumer that makes
	 * the VPSS write a LINEAR de-tiled/decompressed frame to memory. The display
	 * window keeps NV21_TILE_CMP (102) all the way through (proven: HI-V4L2-NI
	 * shows 102 on every node incl. output; the VDP de-tiles only in the HDMI
	 * scanout read, so the cast/WBC captured 102). enDataFormat is "only for
	 * virtual window" (hi_drv_win.h:104) — setting NV21 here is the documented
	 * "consumer asks for linear" lever that forces the VPSS detile/decompress
	 * output node. HI_DRV_WIN_Create forces bMCE=TRUE (decode-to-memory mode),
	 * which the earlier DRV_WIN_Process(bMCE=FALSE) virtual attempt lacked (0
	 * frames). Frames pulled via WIN_AcquireFrame (linear NV21). */
	wattr.bVirtual            = HI_TRUE;
	wattr.enDisp              = HI_DRV_DISPLAY_1;
	wattr.enARCvrs            = HI_DRV_ASP_RAT_MODE_FULL;
	wattr.bUseCropRect        = HI_FALSE;
	wattr.stOutRect.s32X      = 0;
	wattr.stOutRect.s32Y      = 0;
	wattr.stOutRect.s32Width  = 1920;
	wattr.stOutRect.s32Height = 1080;
	wattr.bUserAllocBuffer    = HI_FALSE;	/* VO allocs the NV21 linear pool */
	wattr.u32BufNumber        = 4;
	wattr.enDataFormat        = HI_DRV_PIX_FMT_NV21;	/* LINEAR output */
	/* A virtual window must be created via DRV_WIN_Process with bVirtScreen=TRUE,
	 * bMCE=FALSE — exactly HI_MPI_WIN_Create (mpi_win.c:414). HI_DRV_WIN_Create
	 * leaves bVirtScreen uninitialized and forces bMCE=TRUE, so it fails outright
	 * for bVirtual=TRUE (observed: 0xffffffff). */
	memset(&wc, 0, sizeof(wc));
	wc.WinAttr      = wattr;
	wc.bVirtScreen  = HI_TRUE;
	wc.bMCE         = HI_FALSE;
	ret = DRV_WIN_Process(CMD_WIN_CREATE, &wc);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: DRV_WIN_Process(CREATE virtual) failed (0x%x)\n", ret);
		c->hWin = 0;
		goto err_winput;
	}
	c->hWin = wc.hWindow;

	src.enSrcMode  = HI_ID_VDEC;
	src.hSrc       = c->hPort;		/* VPSS master port handle */
	src.hSecondSrc = HI_INVALID_HANDLE;
	ret = HI_DRV_WIN_SetSource(c->hWin, &src);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: WIN_SetSource failed (0x%x)\n", ret);
		goto err_destroywin;
	}
	ret = HI_DRV_WIN_SetEnable(c->hWin, HI_TRUE);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 win: WIN_SetEnable failed (0x%x)\n", ret);
		goto err_destroywin;
	}

	/* No cast/WBC: a virtual window (enDataFormat=NV21) makes the VPSS write
	 * LINEAR frames into the window's own pool; we pull them with WIN_AcquireFrame
	 * (vp_recv win branch). The cast path is retired — it captured 102 (tiled+CMP)
	 * because the de-tile is downstream of the WBC tap. */
	c->hCast = 0;

	pr_info("hi-v4l2 win: VIRTUAL VO window (NV21 linear) up (hVdec=0x%x hPort=0x%x hWin=0x%x)\n",
		c->hVdec, c->hPort, c->hWin);
	return c;

err_destroywin:
	HI_DRV_WIN_SetEnable(c->hWin, HI_FALSE);
	HI_DRV_WIN_Destroy(c->hWin);
	c->hWin = 0;
err_winput:
	hi_win_subsys_put();
err_destroyport:
	HI_DRV_VDEC_DestroyPort(c->hVdec, c->hPort);
err_bufdeinit:
	HI_DRV_VDEC_ChanBufferDeInit(c->hVdec);
err_freechan:
	HI_DRV_VDEC_FreeChan(c->hVdec);
err_free:
	kfree(c);
	return ERR_PTR(-EIO);
}

/*
 * Variant B (vp): VPSS-managed decode WITHOUT a VO window. Builds the same VPSS
 * HD/MASTER port as the win path but leaves the buffer mode at the default
 * (VPSS_ALLOC_MANAGE — no UserBuf_SetMode), so the VPSS allocates AND processes
 * its own linear NV12 output. Frames are pulled with RecvVpssFrmBuf (the recv
 * that returned frames in the zc probe) and memcpy'd to CAPTURE. recv/release
 * dispatch on c->vpss_recv. Tests whether the VPSS linearizes when driven by a
 * pull consumer alone (no window push, no master-port free-run).
 */
struct hi_vdec_chan *hi_vdec_hal_vp_create(const struct hi_v4l2_fmt *coded,
					   u32 width, u32 height, u32 es_buf_size)
{
	struct hi_vdec_chan *c;
	HI_UNF_AVPLAY_OPEN_OPT_S opt = {0};
	HI_UNF_VCODEC_ATTR_S attr = {0};
	int ret;

	if (!coded || !(coded->flags & HI_V4L2_CODED))
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->coded = coded;
	c->es_buf_size = es_buf_size;
	c->vpss_recv = true;

	opt.enDecType       = HI_UNF_VCODEC_DEC_TYPE_NORMAL;
	opt.enCapLevel      = cap_level_for(width, height);
	opt.enProtocolLevel = prtcl_level_for(coded->vcodec);

	ret = HI_DRV_VDEC_AllocChan(&c->hVdec, &opt);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 vp: AllocChan failed (0x%x)\n", ret);
		goto err_free;
	}

	attr.enType       = coded->vcodec;
	attr.enMode       = HI_UNF_VCODEC_MODE_NORMAL;
	attr.u32ErrCover  = 100;
	attr.u32Priority  = 3;
	attr.bOrderOutput = HI_TRUE;
	if (coded->vcodec == HI_UNF_VCODEC_TYPE_VC1) {
		attr.unExtAttr.stVC1Attr.bAdvancedProfile = HI_TRUE;
		attr.unExtAttr.stVC1Attr.u32CodecVersion  = 8;
	}
	ret = HI_DRV_VDEC_SetChanAttr(c->hVdec, &attr);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 vp: SetChanAttr failed (0x%x)\n", ret);
		goto err_freechan;
	}

	ret = HI_DRV_VDEC_ChanBufferInit(c->hVdec, es_buf_size, HI_INVALID_HANDLE);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 vp: ChanBufferInit failed (0x%x)\n", ret);
		goto err_freechan;
	}

	/* VPSS HD master port, default VPSS_ALLOC_MANAGE (no UserBuf_SetMode). */
	ret = HI_DRV_VDEC_CreatePort(c->hVdec, &c->hPort, VDEC_PORT_HD);
	if (ret != HI_SUCCESS) {
		pr_err("hi-v4l2 vp: CreatePort(HD) failed (0x%x)\n", ret);
		goto err_bufdeinit;
	}
	HI_DRV_VDEC_SetPortType(c->hVdec, c->hPort, VDEC_PORT_TYPE_MASTER);
	HI_DRV_VDEC_EnablePort(c->hVdec, c->hPort);

	/* Force the VPSS node to run (defeat the same-resolution passthrough) so it
	 * de-tiles + de-compresses into a LINEAR NV21 surface. Output 1280x720 is
	 * deliberately != the 1080p input. HW-VALIDATE: render RecvVpssFrmBuf out. */
	{
		HI_DRV_VPSS_PORT_CFG_S pcfg;
		int r = HI_DRV_VPSS_GetPortCfg(c->hPort, &pcfg);

		if (r == HI_SUCCESS) {
			pr_info("hi-v4l2 vp: pre  eFmt=%d outW=%d outH=%d pass=%d\n",
				pcfg.eFormat, pcfg.s32OutputWidth,
				pcfg.s32OutputHeight, pcfg.bPassThrough);
			/* OMX-style force: bPassThrough=FALSE makes
			 * VPSS_INST_SetPortCfg() (via CheckPassThroughForOMX)
			 * clear bNeedTrans -> the VPSS de-tiles even at 1:1
			 * (no scale). Keep output == input resolution. */
			pcfg.eFormat         = HI_DRV_PIX_FMT_NV21;
			pcfg.bPassThrough    = HI_FALSE;
			pcfg.s32OutputWidth  = width;
			pcfg.s32OutputHeight = height;
			r = HI_DRV_VPSS_SetPortCfg(c->hPort, &pcfg);
			pr_info("hi-v4l2 vp: SetPortCfg %dx%d NV21 pass=0 ret=0x%x\n",
				width, height, r);
		} else {
			pr_info("hi-v4l2 vp: GetPortCfg ret=0x%x (hPort not a VPSS port?)\n",
				r);
		}
	}

	pr_info("hi-v4l2 vp: VPSS HD master port (no window) up (hVdec=0x%x hPort=0x%x)\n",
		c->hVdec, c->hPort);
	return c;

err_bufdeinit:
	HI_DRV_VDEC_ChanBufferDeInit(c->hVdec);
err_freechan:
	HI_DRV_VDEC_FreeChan(c->hVdec);
err_free:
	kfree(c);
	return ERR_PTR(-EIO);
}
