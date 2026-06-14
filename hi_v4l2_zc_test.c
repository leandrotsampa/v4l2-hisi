// SPDX-License-Identifier: GPL-2.0
/*
 * Zero-copy Fase C (KI-003) — ISOLATED PROTOTYPE (Passo 2).
 *
 * Proves the VDEC can decode straight into buffers WE provide, via the VPSS
 * user-managed path (HI_DRV_VDEC_UserBuf_* wrappers added to the vendor in
 * Passo 1), WITHOUT touching the production decoder (device_run / RecvFrmBuf).
 *
 * Whole thing is gated by HI_V4L2_ZC_SELFTEST (Makefile flag). Remove the file,
 * the flag, the core.c hooks and hi_dec_dma_dev() once Passo 3 (device_run
 * migration to user-managed buffers) is validated on HW.
 *
 * Trigger from the target (root):
 *     echo /tmp/test.264            > /sys/kernel/debug/hi_v4l2_zc/run
 *     echo "/tmp/clip.264 1280 720" > /sys/kernel/debug/hi_v4l2_zc/run
 * Defaults: 1920x1080, H264 Annex-B (split on AUD start codes). Read the result
 * in /var/log/kern.log: each decoded frame logs its phys_y and whether it
 * landed in one of OUR committed CMA buffers ("ours=1" == zero-copy works).
 *
 * Caveat (documented for the next iteration): hi_vdec_hal_create() also sets up
 * the VFMW-direct STR virtual port. AllocChan additionally creates the VPSS, so
 * committing user buffers + RecvVpssFrmBuf exercises a *different* output than
 * the STR port. If RecvVpssFrmBuf returns nothing, Passo 2b is to create the
 * channel WITHOUT the STR port so the VFMW routes frames to the VPSS only.
 */
#ifdef HI_V4L2_ZC_SELFTEST

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>

#include "hi_v4l2_drv.h"
#include "hi_v4l2_fmt.h"
#include "hi_vdec_hal.h"

#define ZC_BUF_NUM	8
#define ZC_ES_MAX	(64 << 20)	/* sane cap for the test clip */

static struct dentry *zc_dir;

/* Read a whole file into a vmalloc buffer. Returns size, <0 on error. */
static long zc_read_file(const char *path, void **out)
{
	struct file *filp;
	long size, n;
	void *buf;

	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	size = i_size_read(file_inode(filp));
	if (size <= 0 || size > ZC_ES_MAX) {
		filp_close(filp, NULL);
		return -EINVAL;
	}
	buf = vmalloc(size);
	if (!buf) {
		filp_close(filp, NULL);
		return -ENOMEM;
	}
	/* 4.4 signature: kernel_read(file, offset, addr, count). */
	n = kernel_read(filp, 0, buf, size);
	filp_close(filp, NULL);
	if (n != size) {
		vfree(buf);
		return -EIO;
	}
	*out = buf;
	return size;
}

/* Next Annex-B AUD (00 00 00 01 09) at/after @off; -1 if none. */
static long zc_next_aud(const u8 *b, long len, long off)
{
	long i;

	for (i = off; i + 4 < len; i++) {
		if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 0 &&
		    b[i + 3] == 1 && b[i + 4] == 0x09)
			return i;
	}
	return -1;
}

static int zc_is_ours(const u32 *phys, u32 n, u32 y)
{
	u32 i;

	for (i = 0; i < n; i++)
		if (phys[i] == y)
			return 1;
	return 0;
}

static void zc_drain_one(struct hi_vdec_chan *c, const u32 *phys, u32 *frames)
{
	struct hi_vdec_frame f;

	if (hi_vdec_hal_zc_recv(c, &f) != 0)
		return;
	if (f.last)
		return;
	(*frames)++;
	pr_info("hi-v4l2 zc: FRAME #%u %ux%u phys_y=0x%08x stride=%u ours=%d\n",
		*frames, f.width, f.height, f.phys_y, f.stride_y,
		zc_is_ours(phys, ZC_BUF_NUM, f.phys_y));
	hi_vdec_hal_zc_release(c, &f);
}

static int zc_run_one(const char *path, u32 w, u32 h)
{
	const struct hi_v4l2_fmt *fmt;
	struct device *dev = hi_dec_dma_dev();
	struct hi_vdec_chan *c = NULL;
	void *es = NULL;
	void *cpu[ZC_BUF_NUM] = { NULL };
	dma_addr_t dma[ZC_BUF_NUM] = { 0 };
	u32 phys[ZC_BUF_NUM] = { 0 };
	u64 virt[ZC_BUF_NUM] = { 0 };
	long es_len, pos;
	u32 size, stride, i, n_alloc = 0, fed = 0, frames = 0;
	int ret, tries, empties = 0;

	if (!dev) {
		pr_err("hi-v4l2 zc: no CMA device (decoder not registered?)\n");
		return -ENODEV;
	}
	fmt = hi_v4l2_find_fmt(V4L2_PIX_FMT_H264, HI_V4L2_CODED);
	if (!fmt) {
		pr_err("hi-v4l2 zc: H264 coded fmt not found\n");
		return -EINVAL;
	}

	es_len = zc_read_file(path, &es);
	if (es_len < 0) {
		pr_err("hi-v4l2 zc: read '%s' failed (%ld)\n", path, es_len);
		return es_len;
	}
	pr_info("hi-v4l2 zc: ES '%s' %ld bytes, target %ux%u\n",
		path, es_len, w, h);

	stride = ALIGN(w, 16);
	size   = stride * ALIGN(h, 16) * 3 / 2;		/* NV12 */

	for (i = 0; i < ZC_BUF_NUM; i++) {
		cpu[i] = dma_alloc_coherent(dev, size, &dma[i], GFP_KERNEL);
		if (!cpu[i]) {
			pr_err("hi-v4l2 zc: dma_alloc_coherent[%u] %u bytes failed\n",
			       i, size);
			ret = -ENOMEM;
			goto out;
		}
		phys[i] = (u32)dma[i];
		virt[i] = (u64)(unsigned long)cpu[i];
		n_alloc++;
	}
	pr_info("hi-v4l2 zc: %u CMA buffers, size=%u stride=%u phys[0]=0x%08x\n",
		n_alloc, size, stride, phys[0]);

	c = hi_vdec_hal_zc_create(fmt, w, h, 4 << 20);	/* VPSS HD master port */
	if (IS_ERR(c)) {
		ret = PTR_ERR(c);
		c = NULL;
		pr_err("hi-v4l2 zc: hal_zc_create failed (%d)\n", ret);
		goto out;
	}

	ret = hi_vdec_hal_zc_commit(c, phys, virt, ZC_BUF_NUM, size, stride);
	if (ret)
		goto out;

	ret = hi_vdec_hal_start(c);
	if (ret) {
		pr_err("hi-v4l2 zc: start failed (%d)\n", ret);
		goto out;
	}

	/* Feed one Annex-B AU per PutEsBuf, distinct PTS, draining on backpressure. */
	pos = zc_next_aud(es, es_len, 0);
	if (pos < 0)
		pos = 0;			/* no AUD framing: feed as one blob */
	while (pos < es_len) {
		long nxt = zc_next_aud(es, es_len, pos + 5);
		long au  = (nxt < 0 ? es_len : nxt) - pos;

		ret = hi_vdec_hal_feed_es(c, (u8 *)es + pos, au, fed + 1, nxt < 0);
		if (ret == -EAGAIN) {		/* ES ring full: drain, retry AU */
			zc_drain_one(c, phys, &frames);
			usleep_range(2000, 4000);
			continue;
		}
		if (ret < 0) {
			pr_err("hi-v4l2 zc: feed_es failed (%d)\n", ret);
			break;
		}
		fed++;
		if (nxt < 0)
			break;
		pos = nxt;
	}
	hi_vdec_hal_set_eos(c);
	pr_info("hi-v4l2 zc: fed %u AUs, draining...\n", fed);

	/*
	 * The VPSS user-managed path has no phys==0 EOS sentinel (that is a
	 * VFMW-direct/RecvFrmBuf artifact). And since we release each buffer back,
	 * the VPSS happily re-delivers them forever. So bound the drain: stop once
	 * we have collected as many frames as AUs fed, or after a short idle gap.
	 */
	for (tries = 0; tries < 600 && frames < fed; tries++) {
		struct hi_vdec_frame f;

		ret = hi_vdec_hal_zc_recv(c, &f);
		if (ret == -EAGAIN) {
			if (++empties > 40)	/* ~0.2s with nothing new -> done */
				break;
			usleep_range(3000, 6000);
			continue;
		}
		empties = 0;
		if (f.last) {
			pr_info("hi-v4l2 zc: EOS sentinel\n");
			break;
		}
		frames++;
		pr_info("hi-v4l2 zc: FRAME #%u %ux%u phys_y=0x%08x stride=%u ours=%d\n",
			frames, f.width, f.height, f.phys_y, f.stride_y,
			zc_is_ours(phys, ZC_BUF_NUM, f.phys_y));
		hi_vdec_hal_zc_release(c, &f);
	}

	pr_info("hi-v4l2 zc: DONE — %u frames into user buffers (fed %u AUs)\n",
		frames, fed);
	ret = 0;
out:
	if (c)
		hi_vdec_hal_destroy(c);
	for (i = 0; i < n_alloc; i++)
		dma_free_coherent(dev, size, cpu[i], dma[i]);
	if (es)
		vfree(es);
	return ret;
}

static ssize_t zc_run_write(struct file *fp, const char __user *ubuf,
			    size_t cnt, loff_t *ppos)
{
	char kbuf[160], path[128];
	u32 w = 1920, h = 1080;
	int got;

	if (cnt == 0 || cnt >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';

	got = sscanf(kbuf, "%127s %u %u", path, &w, &h);
	if (got < 1)
		return -EINVAL;

	zc_run_one(path, w, h);		/* result goes to the kernel log */
	return cnt;
}

static const struct file_operations zc_run_fops = {
	.owner	= THIS_MODULE,
	.write	= zc_run_write,
};

/*
 * De-tile geometry probe: feed IMAGE_TileTo2D (the vendor de-tiler) a SYNTHETIC
 * input where each byte encodes its own offset, capture the LINEAR output. Then
 * output[y*w+x] = in[src_offset(x,y)] = (pass ? src>>8 : src) & 0xFF, so two
 * passes recover the exact src offset for every linear (x,y) -> the precise
 * tile geometry, WITHOUT decoding video or touching the (wedge-prone) saveyuv.
 *   echo "W H stride is1d pass" > /sys/kernel/debug/hi_v4l2_zc/probe
 *   cat /sys/kernel/debug/hi_v4l2_zc/out   # W*H bytes
 */
struct dt_param { u8 *y8, *yn, *c8, *cn; s32 rbd, sbd; u32 is1d, stride; };
struct dt_addr  { u8 *y8, *yn, *c8, *cn; };
extern void IMAGE_TileTo2D(struct dt_param *p, u32 w, u32 h, struct dt_addr *a);

static u8 *g_probe_out;
static size_t g_probe_len;

static ssize_t dt_probe_write(struct file *f, const char __user *ubuf,
			      size_t cnt, loff_t *ppos)
{
	char kbuf[96];
	u32 w = 128, h = 32, stride = 128, is1d = 0, pass = 0, plane = 0, i, ah, insz, outsz;
	struct dt_param p;
	struct dt_addr a;
	u8 *iny, *inc, *outy, *outc, *pat, *outp;

	if (cnt == 0 || cnt >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, cnt))
		return -EFAULT;
	kbuf[cnt] = '\0';
	sscanf(kbuf, "%u %u %u %u %u %u", &w, &h, &stride, &is1d, &pass, &plane);

	ah = ALIGN(h, 16);
	outsz = 1u << 20;	/* read 1 MB of outy to locate the C region (after Y) */
	/* over-allocate generously: IMAGE_TileTo2D's real addressing exceeds the
	 * naive 64x16 model (it OOB-crashed at exact sizes). 8 MB >> any small-dim
	 * tile span; the input is FULLY patterned so even "beyond" reads land on
	 * valid pattern bytes and reveal the true src offset. */
#define DT_BIG (32u << 20)
	insz = DT_BIG;
	iny = vmalloc(insz); inc = vmalloc(insz);
	outy = vmalloc(insz); outc = vmalloc(insz);
	if (!iny || !inc || !outy || !outc) {
		vfree(iny); vfree(inc); vfree(outy); vfree(outc);
		return -ENOMEM;
	}
	pat  = plane ? inc : iny;	/* pattern the probed plane's INPUT */
	outp = outy;	/* IMAGE_TileTo2D writes BOTH Y and C into the a.y8 buffer
			 * (it recomputes a.c8 = a.y8 + Ysize internally); for the C
			 * probe the chroma lands in outy AFTER the (zeroed) Y region. */
	for (i = 0; i < insz; i++)
		pat[i] = (i >> (pass * 8)) & 0xFF;	/* pass 0/1/2 = byte 0/1/2 */
	memset(plane ? iny : inc, 0, insz);
	memset(outy, 0, insz);
	memset(outc, 0, insz);
	(void)ah;

	memset(&p, 0, sizeof(p));
	memset(&a, 0, sizeof(a));
	p.y8 = iny; p.c8 = inc; p.rbd = 8; p.sbd = 8;
	p.is1d = is1d; p.stride = stride;
	a.y8 = outy; a.c8 = outc;
	IMAGE_TileTo2D(&p, w, h, &a);

	vfree(g_probe_out);
	g_probe_out = vmalloc(outsz);
	if (g_probe_out) {
		memcpy(g_probe_out, outp, outsz);
		g_probe_len = outsz;
	} else {
		g_probe_len = 0;
	}
	pr_info("hi-v4l2 dtprobe: w=%u h=%u stride=%u is1d=%u pass=%u -> %u bytes\n",
		w, h, stride, is1d, pass, g_probe_len ? outsz : 0);

	vfree(iny); vfree(inc); vfree(outy); vfree(outc);
	return cnt;
}

static ssize_t dt_probe_read(struct file *f, char __user *ubuf,
			     size_t cnt, loff_t *ppos)
{
	return simple_read_from_buffer(ubuf, cnt, ppos, g_probe_out, g_probe_len);
}

static const struct file_operations dt_probe_fops = {
	.owner = THIS_MODULE,
	.write = dt_probe_write,
};
static const struct file_operations dt_out_fops = {
	.owner = THIS_MODULE,
	.read  = dt_probe_read,
};

int hi_zc_test_register(void)
{
	zc_dir = debugfs_create_dir("hi_v4l2_zc", NULL);
	if (IS_ERR_OR_NULL(zc_dir)) {
		pr_warn("hi-v4l2 zc: debugfs_create_dir failed\n");
		zc_dir = NULL;
		return -ENODEV;
	}
	debugfs_create_file("run", 0200, zc_dir, NULL, &zc_run_fops);
	debugfs_create_file("probe", 0200, zc_dir, NULL, &dt_probe_fops);
	debugfs_create_file("out", 0400, zc_dir, NULL, &dt_out_fops);
	pr_info("hi-v4l2 zc: probe ready — echo <es-path> [w h] > /sys/kernel/debug/hi_v4l2_zc/run\n");
	return 0;
}

void hi_zc_test_unregister(void)
{
	debugfs_remove_recursive(zc_dir);
	zc_dir = NULL;
	vfree(g_probe_out);
	g_probe_out = NULL;
}

#endif /* HI_V4L2_ZC_SELFTEST */
