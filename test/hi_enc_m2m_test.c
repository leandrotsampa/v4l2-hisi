// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal V4L2 m2m encoder harness for hi_v4l2 (/dev/video1).
 *
 * Mirror of hi_dec_m2m_test.c, reversed:
 *   S_FMT(out=NV12) S_FMT(cap=H264) -> REQBUFS both -> mmap ->
 *   feed 1 raw NV12 frame per OUTPUT buffer -> STREAMON both ->
 *   poll/DQBUF capture (H264 AU) -> append to file -> requeue.
 *
 * Input is raw NV12, one frame = OUTPUT sizeimage bytes (as reported by the
 * driver after S_FMT). The easiest matching source is the decoder's own
 * output: decode a clip with hi_dec_m2m_test first, then feed that .nv12 here
 * (same driver -> same raw frame geometry, no stride/alignment mismatch).
 *
 * Build (host): arm-linux-gnueabihf-gcc -std=gnu99 -O2 -static \
 *                   -o hi_enc_m2m_test hi_enc_m2m_test.c
 * Run (target): ./hi_enc_m2m_test /dev/video1 in.nv12 out.h264 1920 1080
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

/* Buffer counts default low: the encoder OUTPUT is dma-contig and the SoC's
 * CMA/MMZ carveout is tiny and largely consumed by the SDK display stack
 * (KI-013), so few/small buffers are needed to even REQBUFS. Override with
 * env HI_NB (e.g. HI_NB=2). */
#ifndef NB_OUT
#define NB_OUT 2
#endif
#ifndef NB_CAP
#define NB_CAP 2
#endif
#ifndef V4L2_PIX_FMT_NV12
#define V4L2_PIX_FMT_NV12 v4l2_fourcc('N', 'V', '1', '2')
#endif
#ifndef V4L2_BUF_FLAG_LAST
#define V4L2_BUF_FLAG_LAST 0x00100000	/* last buffer of an EOS drain */
#endif
#ifndef V4L2_PIX_FMT_HEVC
#define V4L2_PIX_FMT_HEVC v4l2_fourcc('H', 'E', 'V', 'C')
#endif

/* map an encoder output codec name (argv[6], default h264) to its fourcc */
static unsigned enc_fourcc(const char *s)
{
	if (!s || !strcmp(s, "h264")) return V4L2_PIX_FMT_H264;
	if (!strcmp(s, "hevc") || !strcmp(s, "h265")) return V4L2_PIX_FMT_HEVC;
	if (!strcmp(s, "jpeg")) return V4L2_PIX_FMT_JPEG;
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

int main(int argc, char **argv)
{
	if (argc < 6) {
		fprintf(stderr, "usage: %s /dev/videoN in.nv12 out W H [codec]\n"
			"  codec: h264(def) hevc jpeg\n", argv[0]);
		return 1;
	}
	const char *dev = argv[1], *inf = argv[2], *outf = argv[3];
	int W = atoi(argv[4]), H = atoi(argv[5]);
	unsigned out_fourcc = enc_fourcc(argc > 6 ? argv[6] : "h264");

	FILE *inF = fopen(inf, "rb");
	if (!inF) { perror("fopen in"); return 1; }

	int fd = open(dev, O_RDWR);
	if (fd < 0) { perror("open dev"); return 1; }

	/* formats: OUTPUT = raw NV12, CAPTURE = coded H264 */
	struct v4l2_format f;
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	f.fmt.pix.width = W; f.fmt.pix.height = H;
	f.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &f, "S_FMT out") < 0) return 1;
	unsigned in_size = f.fmt.pix.sizeimage;
	printf("OUTPUT(NV12) sizeimage=%u bytesperline=%u (W=%u H=%u)\n",
	       in_size, f.fmt.pix.bytesperline, f.fmt.pix.width, f.fmt.pix.height);

	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f.fmt.pix.width = W; f.fmt.pix.height = H;
	f.fmt.pix.pixelformat = out_fourcc;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &f, "S_FMT cap") < 0) return 1;
	printf("CAPTURE(0x%08x) sizeimage=%u\n", out_fourcc, f.fmt.pix.sizeimage);

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
	int out_q = 0, aus = 0, eos = 0, nonempty = 0, stopped = 0, last = 0;
	long total = 0;
	unsigned long fed = 0;	/* frame counter -> distinct, nonzero timestamps */

	/* read one NV12 frame into OUTPUT buffer i; returns bytes read (0 = EOF).
	 * Each frame gets a distinct, nonzero timestamp: the encoder dedups input
	 * by PTS (VENC_DRV_EflQueryChn_X "don't re-get"), so all-zero timestamps
	 * make it skip every frame and emit nothing. */
	#define FEED(i) do {                                                   \
		size_t rd = fread(ob[i].start, 1,                              \
				  in_size < ob[i].length ? in_size : ob[i].length, inF); \
		if (rd < in_size) { eos = 1; }                                 \
		if (rd > 0) {                                                  \
			struct v4l2_buffer b; memset(&b, 0, sizeof(b));        \
			b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;                   \
			b.memory = V4L2_MEMORY_MMAP; b.index = (i);            \
			b.bytesused = rd;                                      \
			fed++;                                                 \
			b.timestamp.tv_sec  = fed / 30;                        \
			b.timestamp.tv_usec = (fed % 30) * 33333;              \
			xioctl(fd, VIDIOC_QBUF, &b, "QBUF out");               \
			out_q++;                                               \
		}                                                              \
	} while (0)

	/* prime OUTPUT buffers with the first frames */
	for (int i = 0; i < nout && !eos; i++)
		FEED(i);

	for (int iter = 0; iter < 5000; iter++) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN | POLLOUT };
		int pr = poll(&pfd, 1, 1000);
		if (pr <= 0) { printf("poll timeout/end (aus=%d)\n", aus); break; }

		if (pfd.revents & POLLIN) { /* coded AU ready */
			struct v4l2_buffer b; memset(&b, 0, sizeof(b));
			b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
			if (xioctl(fd, VIDIOC_DQBUF, &b, "DQBUF cap") == 0) {
				if (b.bytesused > 0) {
					aus++;
					nonempty++;
					total += b.bytesused;
					fwrite(cb[b.index].start, 1, b.bytesused, of);
				}
				if (b.flags & V4L2_BUF_FLAG_LAST)
					last = 1;
				if (aus <= 8 || b.bytesused == 0)
					printf("  cap AU#%d idx=%u bytesused=%u%s\n",
					       aus, b.index, b.bytesused,
					       (b.flags & V4L2_BUF_FLAG_LAST) ? " [LAST]" : "");
				if (!last)
					xioctl(fd, VIDIOC_QBUF, &b, "re-QBUF cap");
			}
		}
		if (pfd.revents & POLLOUT) { /* OUTPUT buffer free -> refill */
			struct v4l2_buffer b; memset(&b, 0, sizeof(b));
			b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory = V4L2_MEMORY_MMAP;
			if (xioctl(fd, VIDIOC_DQBUF, &b, "DQBUF out") == 0) {
				out_q--;
				if (!eos)
					FEED(b.index);
			}
		}
		/* All frames queued: drain the encoder (flush the last AU). */
		if (eos && !stopped) {
			struct v4l2_encoder_cmd ec;
			memset(&ec, 0, sizeof(ec));
			ec.cmd = V4L2_ENC_CMD_STOP;
			xioctl(fd, VIDIOC_ENCODER_CMD, &ec, "ENC_CMD_STOP");
			stopped = 1;
			printf("EOS sent (AUs so far=%d), draining...\n", aus);
		}
		if (last) {
			printf("LAST buffer received, drain complete\n");
			break;
		}
	}

	t = V4L2_BUF_TYPE_VIDEO_OUTPUT;  xioctl(fd, VIDIOC_STREAMOFF, &t, "STREAMOFF out");
	t = V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd, VIDIOC_STREAMOFF, &t, "STREAMOFF cap");
	fclose(of); fclose(inF);
	printf("DONE: AUs=%d non-empty=%d total=%ld bytes -> %s\n",
	       aus, nonempty, total, outf);
	close(fd);
	return 0;
}
