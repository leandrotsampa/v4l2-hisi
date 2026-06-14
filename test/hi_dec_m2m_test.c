// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal V4L2 m2m decoder harness for hi_v4l2 (/dev/video0).
 *
 * v4l2-ctl 1.16.3 does not drive our m2m decode (stops before REQBUFS, see
 * KI-014), so this tool does the handshake explicitly:
 *   S_FMT(out=H264) S_FMT(cap=NV12) -> REQBUFS both -> mmap ->
 *   feed 1 Annex-B AU per OUTPUT buffer -> STREAMON both ->
 *   poll/DQBUF capture (NV12) -> write to file -> requeue.
 *
 * Splits the input on AUD start codes (00 00 00 01 09), which gst
 * 'h264parse ! video/x-h264,alignment=au' inserts, so each OUTPUT buffer
 * carries exactly one access unit (matches the PHASE1 1-AU->1-frame model).
 *
 * Build (host): arm-linux-gnueabihf-gcc -O2 -o hi_dec_m2m_test hi_dec_m2m_test.c
 * Run (target): ./hi_dec_m2m_test /dev/video0 in.264 out.nv12 1920 1080
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define NB_OUT 4
#define NB_CAP 4
#ifndef V4L2_PIX_FMT_NV12
#define V4L2_PIX_FMT_NV12 v4l2_fourcc('N', 'V', '1', '2')
#endif
#ifndef V4L2_BUF_FLAG_LAST
#define V4L2_BUF_FLAG_LAST 0x00100000	/* last buffer of an EOS drain */
#endif
/* fourccs that may be absent from the 4.4 UAPI (defined like the driver does) */
#ifndef V4L2_PIX_FMT_HEVC
#define V4L2_PIX_FMT_HEVC v4l2_fourcc('H', 'E', 'V', 'C')
#endif
#ifndef V4L2_PIX_FMT_VP8
#define V4L2_PIX_FMT_VP8  v4l2_fourcc('V', 'P', '8', '0')
#endif
#ifndef V4L2_PIX_FMT_VP9
#define V4L2_PIX_FMT_VP9  v4l2_fourcc('V', 'P', '9', '0')
#endif
#ifndef V4L2_PIX_FMT_VC1_ANNEX_G
#define V4L2_PIX_FMT_VC1_ANNEX_G v4l2_fourcc('V', 'C', '1', 'G')
#endif

/* map a codec name (argv[6], default h264) to its OUTPUT fourcc */
static unsigned codec_fourcc(const char *s, int *is_h264)
{
	*is_h264 = 0;
	if (!s || !strcmp(s, "h264")) { *is_h264 = 1; return V4L2_PIX_FMT_H264; }
	if (!strcmp(s, "hevc") || !strcmp(s, "h265")) return V4L2_PIX_FMT_HEVC;
	if (!strcmp(s, "mpeg2")) return V4L2_PIX_FMT_MPEG2;
	if (!strcmp(s, "mpeg4")) return V4L2_PIX_FMT_MPEG4;
	if (!strcmp(s, "h263"))  return V4L2_PIX_FMT_H263;
	if (!strcmp(s, "mjpeg")) return V4L2_PIX_FMT_MJPEG;
	if (!strcmp(s, "vp8"))   return V4L2_PIX_FMT_VP8;
	if (!strcmp(s, "vp9"))   return V4L2_PIX_FMT_VP9;
	if (!strcmp(s, "vc1"))   return V4L2_PIX_FMT_VC1_ANNEX_G;
	return V4L2_PIX_FMT_H264;
}

struct buf { void *start; size_t length; };

static int xioctl(int fd, unsigned long req, void *arg, const char *name)
{
	int r = ioctl(fd, req, arg);
	if (r < 0)
		fprintf(stderr, "%s: %s (errno=%d)\n", name, strerror(errno), errno);
	return r;
}

/* Load whole file. */
static unsigned char *load(const char *path, size_t *len)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror("fopen"); return NULL; }
	fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
	unsigned char *b = malloc(n);
	if (fread(b, 1, n, f) != (size_t)n) { fclose(f); free(b); return NULL; }
	fclose(f); *len = n; return b;
}

/* Find next AUD start code (00 00 00 01 09) at or after off; -1 if none. */
static long next_au(const unsigned char *b, size_t len, size_t off)
{
	for (size_t i = off; i + 4 < len; i++)
		if (b[i] == 0 && b[i+1] == 0 && b[i+2] == 0 &&
		    b[i+3] == 1 && b[i+4] == 0x09)
			return (long)i;
	return -1;
}

/* Chunk mode for codecs without H264 AUD framing (MPEG2/4, HEVC, MJPEG, VP8...):
 * the VFMW stream parser finds real frame boundaries itself, so feeding fixed
 * chunks just keeps the stream buffer fed across multiple device_run() calls. */
#define CHUNK (64 * 1024)
static int g_chunk_mode;
static int g_mjpeg;	/* one JPEG (SOI..next SOI) per OUTPUT buffer */
/* next slice boundary at/after off; -1 when the stream is exhausted */
static long next_slice(const unsigned char *b, size_t len, size_t off)
{
	if (g_mjpeg) {	/* next SOI marker 0xFFD8 after this frame */
		for (size_t i = off + 2; i + 1 < len; i++)
			if (b[i] == 0xFF && b[i+1] == 0xD8)
				return (long)i;
		return -1;
	}
	if (g_chunk_mode) {
		long n = (long)off + CHUNK;
		return (n >= (long)len) ? -1 : n;
	}
	return next_au(b, len, off + 5);
}

static int g_ivf;	/* IVF container (VP8/VP9): 32B file hdr + 12B/frame hdr */
/* Resolve the next frame to feed: sets *doff/*dlen to the payload to copy and
 * returns the next position (-1 at end). Handles IVF (skip the 12B frame hdr,
 * size is in it) and the other framings (payload = [off, next_slice)). */
static long frame_at(const unsigned char *b, size_t len, size_t off,
		     size_t *doff, size_t *dlen)
{
	if (g_ivf) {
		if (off + 12 > len) return -1;
		unsigned sz = b[off] | (b[off+1]<<8) | (b[off+2]<<16) |
			      ((unsigned)b[off+3]<<24);
		*doff = off + 12;
		*dlen = sz;
		long next = (long)off + 12 + sz;
		return (next + 12 > (long)len) ? -1 : next;
	}
	*doff = off;
	long next = next_slice(b, len, off);
	*dlen = (next < 0 ? len : (size_t)next) - off;
	return next;
}

int main(int argc, char **argv)
{
	if (argc < 6) {
		fprintf(stderr, "usage: %s /dev/videoN in out.nv12 W H [codec]\n"
			"  codec: h264(def) hevc mpeg2 mpeg4 h263 mjpeg vp8 vp9 vc1\n", argv[0]);
		return 1;
	}
	const char *dev = argv[1], *inf = argv[2], *outf = argv[3];
	int W = atoi(argv[4]), H = atoi(argv[5]);
	int is_h264;
	unsigned coded_fourcc = codec_fourcc(argc > 6 ? argv[6] : "h264", &is_h264);

	size_t es_len; unsigned char *es = load(inf, &es_len);
	if (!es) return 1;
	printf("ES %s: %zu bytes\n", inf, es_len);

	int fd = open(dev, O_RDWR);
	if (fd < 0) { perror("open dev"); return 1; }

	/* formats */
	struct v4l2_format f;
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	f.fmt.pix.width = W; f.fmt.pix.height = H;
	f.fmt.pix.pixelformat = coded_fourcc;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &f, "S_FMT out") < 0) return 1;

	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f.fmt.pix.width = W; f.fmt.pix.height = H;
	f.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &f, "S_FMT cap") < 0) return 1;
	unsigned cap_size = f.fmt.pix.sizeimage;
	printf("CAPTURE sizeimage=%u (W=%u H=%u)\n", cap_size, f.fmt.pix.width, f.fmt.pix.height);

	/* reqbufs */
	struct v4l2_requestbuffers rb;
	memset(&rb, 0, sizeof(rb));
	rb.count = NB_OUT; rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; rb.memory = V4L2_MEMORY_MMAP;
	if (xioctl(fd, VIDIOC_REQBUFS, &rb, "REQBUFS out") < 0) return 1;
	int nout = rb.count;
	memset(&rb, 0, sizeof(rb));
	rb.count = NB_CAP; rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory = V4L2_MEMORY_MMAP;
	if (xioctl(fd, VIDIOC_REQBUFS, &rb, "REQBUFS cap") < 0) return 1;
	int ncap = rb.count;
	printf("bufs: out=%d cap=%d\n", nout, ncap);

	struct buf ob[NB_OUT], cb[NB_CAP];
	for (int i = 0; i < nout; i++) {
		struct v4l2_buffer b; memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory = V4L2_MEMORY_MMAP; b.index = i;
		if (xioctl(fd, VIDIOC_QUERYBUF, &b, "QUERYBUF out") < 0) return 1;
		ob[i].length = b.length;
		ob[i].start = mmap(NULL, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, b.m.offset);
	}
	for (int i = 0; i < ncap; i++) {
		struct v4l2_buffer b; memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
		if (xioctl(fd, VIDIOC_QUERYBUF, &b, "QUERYBUF cap") < 0) return 1;
		cb[i].length = b.length;
		cb[i].start = mmap(NULL, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, b.m.offset);
	}

	/* queue all capture buffers */
	for (int i = 0; i < ncap; i++) {
		struct v4l2_buffer b; memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
		xioctl(fd, VIDIOC_QBUF, &b, "QBUF cap");
	}

	/* streamon both */
	int t;
	t = V4L2_BUF_TYPE_VIDEO_OUTPUT;  xioctl(fd, VIDIOC_STREAMON, &t, "STREAMON out");
	t = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd, VIDIOC_STREAMON, &t, "STREAMON cap");

	FILE *of = fopen(outf, "wb");
	/* H264 splits on AUD; every other codec uses fixed chunks (the VFMW parser
	 * finds real frame boundaries itself). */
	long pos = 0;
	if (es_len > 32 && !memcmp(es, "DKIF", 4)) {
		g_ivf = 1; pos = 32;	/* IVF container (VP8/VP9) */
	} else if (is_h264) {
		pos = next_au(es, es_len, 0);
		if (pos < 0) { pos = 0; g_chunk_mode = 1; }
	} else {
		g_chunk_mode = 1;
		if (coded_fourcc == V4L2_PIX_FMT_MJPEG) g_mjpeg = 1;
	}
	printf("framing: %s\n", g_ivf ? "ivf" : g_mjpeg ? "mjpeg-SOI" :
	       g_chunk_mode ? "chunk" : "H264-AUD");
	int out_q = 0, frames = 0, eos = 0, nonempty = 0, stopped = 0, last = 0;

	/* prime: queue OUTPUT buffers with the first AUs */
	for (int i = 0; i < nout && !eos; i++) {
		size_t doff, aul;
		long nxt = frame_at(es, es_len, pos, &doff, &aul);
		if (aul > ob[i].length) aul = ob[i].length;
		memcpy(ob[i].start, es + doff, aul);
		struct v4l2_buffer b; memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory = V4L2_MEMORY_MMAP;
		b.index = i; b.bytesused = aul;
		xioctl(fd, VIDIOC_QBUF, &b, "QBUF out");
		out_q++;
		if (nxt < 0) { eos = 1; break; }
		pos = nxt;
	}

	/* loop */
	for (int iter = 0; iter < 2000; iter++) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN | POLLOUT };
		int pr = poll(&pfd, 1, 1000);
		if (pr <= 0) { printf("poll timeout/end (frames=%d)\n", frames); break; }

		if (pfd.revents & POLLIN) { /* capture frame ready */
			struct v4l2_buffer b; memset(&b, 0, sizeof(b));
			b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
			if (xioctl(fd, VIDIOC_DQBUF, &b, "DQBUF cap") == 0) {
				if (b.bytesused > 0) {
					frames++;
					nonempty++;
					fwrite(cb[b.index].start, 1, b.bytesused, of);
				}
				if (b.flags & V4L2_BUF_FLAG_LAST)
					last = 1;
				if (frames <= 6 || b.bytesused == 0)
					printf("  cap frame#%d idx=%u bytesused=%u%s\n",
					       frames, b.index, b.bytesused,
					       (b.flags & V4L2_BUF_FLAG_LAST) ? " [LAST]" : "");
				if (!last)
					xioctl(fd, VIDIOC_QBUF, &b, "re-QBUF cap");
			}
		}
		if (pfd.revents & POLLOUT) { /* output buffer free -> refill */
			struct v4l2_buffer b; memset(&b, 0, sizeof(b));
			b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory = V4L2_MEMORY_MMAP;
			if (xioctl(fd, VIDIOC_DQBUF, &b, "DQBUF out") == 0) {
				out_q--;
				if (!eos) {
					size_t doff, aul;
					long nxt = frame_at(es, es_len, pos, &doff, &aul);
					if (aul > ob[b.index].length) aul = ob[b.index].length;
					memcpy(ob[b.index].start, es + doff, aul);
					b.bytesused = aul;
					xioctl(fd, VIDIOC_QBUF, &b, "QBUF out");
					out_q++;
					if (nxt < 0) eos = 1; else pos = nxt;
				}
			}
		}
		/* Signal EOS as soon as the last AU is queued (while the VFMW is
		 * still actively reading the stream) so the end-of-stream packet is
		 * delivered in time to flush the full reorder buffer. */
		if (eos && !stopped) {
			struct v4l2_decoder_cmd dc;
			memset(&dc, 0, sizeof(dc));
			dc.cmd = V4L2_DEC_CMD_STOP;
			xioctl(fd, VIDIOC_DECODER_CMD, &dc, "DEC_CMD_STOP");
			stopped = 1;
			printf("EOS sent (frames so far=%d), draining...\n", frames);
		}
		if (last) {
			printf("LAST buffer received, drain complete\n");
			break;
		}
	}

	t = V4L2_BUF_TYPE_VIDEO_OUTPUT;  xioctl(fd, VIDIOC_STREAMOFF, &t, "STREAMOFF out");
	t = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd, VIDIOC_STREAMOFF, &t, "STREAMOFF cap");
	fclose(of);
	printf("DONE: frames=%d non-empty=%d -> %s\n", frames, nonempty, outf);
	close(fd);
	return 0;
}
