// SPDX-License-Identifier: GPL-2.0
/*
 * Zero-copy decode->encode chain test for hi_v4l2.
 *
 * Proves the STANDARD V4L2 dma-buf path (no ION, no custom allocator):
 *   /dev/video0 (decoder) CAPTURE buffers (vb2-dma-contig) are exported with
 *   VIDIOC_EXPBUF; each dma-buf fd is fed to /dev/video1 (encoder) OUTPUT via
 *   V4L2_MEMORY_DMABUF. The decoded NV12 frame goes straight from the decoder
 *   to the encoder with NO userspace copy.
 *
 * Flow (serial, one frame in flight -- enough to prove the path):
 *   decode 1 AU -> DQBUF dec CAPTURE (frame in buf i) ->
 *   QBUF enc OUTPUT (memory=DMABUF, m.fd = expfd[i]) -> encode ->
 *   DQBUF enc CAPTURE (H264) -> write; DQBUF enc OUTPUT -> QBUF dec CAPTURE i.
 *
 * Build (host): arm-linux-gnueabihf-gcc -std=gnu99 -O2 -static \
 *                   -o hi_dmabuf_chain_test hi_dmabuf_chain_test.c
 * Run (target): ./hi_dmabuf_chain_test /dev/video0 /dev/video1 in.264 out.h264 W H
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

#define NB 4
#ifndef V4L2_PIX_FMT_NV12
#define V4L2_PIX_FMT_NV12 v4l2_fourcc('N','V','1','2')
#endif
#ifndef V4L2_BUF_FLAG_LAST
#define V4L2_BUF_FLAG_LAST 0x00100000
#endif

static int xioctl(int fd, unsigned long req, void *arg, const char *n)
{
	int r = ioctl(fd, req, arg);
	if (r < 0) fprintf(stderr, "%s: %s (errno=%d)\n", n, strerror(errno), errno);
	return r;
}
static unsigned char *load(const char *p, size_t *len)
{
	FILE *f = fopen(p, "rb"); if (!f) { perror("fopen"); return NULL; }
	fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
	unsigned char *b = malloc(n);
	if (fread(b, 1, n, f) != (size_t)n) { fclose(f); free(b); return NULL; }
	fclose(f); *len = n; return b;
}
static long next_au(const unsigned char *b, size_t len, size_t off)
{
	for (size_t i = off; i + 4 < len; i++)
		if (!b[i] && !b[i+1] && !b[i+2] && b[i+3]==1 && b[i+4]==0x09)
			return (long)i;
	return -1;
}
static void s_fmt(int fd, int type, int w, int h, unsigned fourcc)
{
	struct v4l2_format f; memset(&f, 0, sizeof(f));
	f.type = type; f.fmt.pix.width = w; f.fmt.pix.height = h;
	f.fmt.pix.pixelformat = fourcc; f.fmt.pix.field = V4L2_FIELD_NONE;
	xioctl(fd, VIDIOC_S_FMT, &f, "S_FMT");
}
static int reqbufs(int fd, int type, int mem, int count)
{
	struct v4l2_requestbuffers rb; memset(&rb, 0, sizeof(rb));
	rb.count = count; rb.type = type; rb.memory = mem;
	if (xioctl(fd, VIDIOC_REQBUFS, &rb, "REQBUFS") < 0) return -1;
	return rb.count;
}
static void streamon(int fd, int type) { int t=type; xioctl(fd, VIDIOC_STREAMON, &t, "STREAMON"); }

int main(int argc, char **argv)
{
	if (argc < 7) {
		fprintf(stderr, "usage: %s /dev/videoDEC /dev/videoENC in.264 out.h264 W H\n", argv[0]);
		return 1;
	}
	const char *decn=argv[1], *encn=argv[2], *inf=argv[3], *outf=argv[4];
	int W=atoi(argv[5]), H=atoi(argv[6]);
	size_t eslen; unsigned char *es = load(inf, &eslen);
	if (!es) return 1;

	int dec = open(decn, O_RDWR), enc = open(encn, O_RDWR);
	if (dec < 0 || enc < 0) { perror("open"); return 1; }

	/* ---- decoder formats + buffers ---- */
	s_fmt(dec, V4L2_BUF_TYPE_VIDEO_OUTPUT,  W, H, V4L2_PIX_FMT_H264);
	s_fmt(dec, V4L2_BUF_TYPE_VIDEO_CAPTURE, W, H, V4L2_PIX_FMT_NV12);
	int ndo = reqbufs(dec, V4L2_BUF_TYPE_VIDEO_OUTPUT,  V4L2_MEMORY_MMAP, NB);
	int ndc = reqbufs(dec, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, NB);
	if (ndo < 1 || ndc < 1) { fprintf(stderr, "dec reqbufs failed\n"); return 1; }

	/* mmap decoder OUTPUT (to feed coded AUs) */
	struct { void *p; size_t l; } dob[NB];
	for (int i=0;i<ndo;i++){ struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory=V4L2_MEMORY_MMAP; b.index=i;
		xioctl(dec,VIDIOC_QUERYBUF,&b,"dec QUERYBUF out"); dob[i].l=b.length;
		dob[i].p=mmap(0,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,dec,b.m.offset); }

	/* EXPORT each decoder CAPTURE buffer as a dma-buf fd (the zero-copy handle) */
	int expfd[NB];
	for (int i=0;i<ndc;i++){ struct v4l2_exportbuffer eb={0};
		eb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; eb.index=i; eb.flags=O_RDWR;
		if (xioctl(dec,VIDIOC_EXPBUF,&eb,"dec EXPBUF")<0){ fprintf(stderr,"EXPBUF failed -> no dma-buf export\n"); return 1; }
		expfd[i]=eb.fd;
		printf("dec CAPTURE[%d] exported as dma-buf fd=%d\n", i, eb.fd);
	}

	/* ---- encoder formats + buffers (OUTPUT imports the decoder dma-bufs) ---- */
	s_fmt(enc, V4L2_BUF_TYPE_VIDEO_OUTPUT,  W, H, V4L2_PIX_FMT_NV12);
	s_fmt(enc, V4L2_BUF_TYPE_VIDEO_CAPTURE, W, H, V4L2_PIX_FMT_H264);
	int neo = reqbufs(enc, V4L2_BUF_TYPE_VIDEO_OUTPUT,  V4L2_MEMORY_DMABUF, ndc);
	int nec = reqbufs(enc, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, NB);
	if (neo < 1 || nec < 1) { fprintf(stderr, "enc reqbufs failed (DMABUF import?)\n"); return 1; }

	struct { void *p; size_t l; } ecb[NB];
	for (int i=0;i<nec;i++){ struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP; b.index=i;
		xioctl(enc,VIDIOC_QUERYBUF,&b,"enc QUERYBUF cap"); ecb[i].l=b.length;
		ecb[i].p=mmap(0,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,enc,b.m.offset);
		xioctl(enc,VIDIOC_QBUF,&b,"enc QBUF cap"); }

	/* queue all decoder CAPTURE buffers */
	for (int i=0;i<ndc;i++){ struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP; b.index=i;
		xioctl(dec,VIDIOC_QBUF,&b,"dec QBUF cap"); }

	streamon(dec, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	streamon(dec, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	streamon(enc, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	streamon(enc, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	FILE *of = fopen(outf, "wb");
	long pos = next_au(es, eslen, 0); if (pos < 0) pos = 0;
	int dq=0, fed=0, encoded=0, eos=0;
	unsigned framesz = W*((H+15)&~15)*3/2;  /* NV12, 16-aligned height */

	/* prime decoder OUTPUT */
	for (int i=0;i<ndo && !eos;i++){
		long nxt=next_au(es,eslen,pos+5); size_t aul=(nxt<0?eslen:(size_t)nxt)-pos;
		if (aul>dob[i].l) aul=dob[i].l; memcpy(dob[i].p, es+pos, aul);
		struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory=V4L2_MEMORY_MMAP; b.index=i; b.bytesused=aul;
		b.timestamp.tv_usec = (++fed)*33333;
		xioctl(dec,VIDIOC_QBUF,&b,"dec QBUF out");
		if (nxt<0){eos=1;break;} pos=nxt;
	}

	for (int it=0; it<4000; it++){
		struct pollfd pfd[2] = { {dec, POLLIN|POLLOUT,0}, {enc, POLLIN|POLLOUT,0} };
		if (poll(pfd, 2, 1000) <= 0){ printf("poll end (encoded=%d)\n", encoded); break; }

		/* decoder produced a frame -> hand its dma-buf to the encoder OUTPUT */
		if (pfd[0].revents & POLLIN){
			struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
			if (xioctl(dec,VIDIOC_DQBUF,&b,"dec DQBUF cap")==0){
				dq++;
				if (b.bytesused>0 && !(b.flags & V4L2_BUF_FLAG_LAST)){
					struct v4l2_buffer e={0}; e.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;
					e.memory=V4L2_MEMORY_DMABUF; e.index=b.index; e.m.fd=expfd[b.index];
					e.bytesused=framesz; e.timestamp=b.timestamp;
					xioctl(enc,VIDIOC_QBUF,&e,"enc QBUF out(DMABUF)");  /* zero-copy! */
				} else {
					/* empty/LAST: just recycle to the decoder */
					struct v4l2_buffer q=b; xioctl(dec,VIDIOC_QBUF,&q,"dec reQBUF cap");
				}
			}
		}
		/* decoder OUTPUT free -> refill with next AU */
		if (pfd[0].revents & POLLOUT){
			struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_OUTPUT; b.memory=V4L2_MEMORY_MMAP;
			if (xioctl(dec,VIDIOC_DQBUF,&b,"dec DQBUF out")==0 && !eos){
				long nxt=next_au(es,eslen,pos+5); size_t aul=(nxt<0?eslen:(size_t)nxt)-pos;
				if (aul>dob[b.index].l) aul=dob[b.index].l; memcpy(dob[b.index].p, es+pos, aul);
				b.bytesused=aul; b.timestamp.tv_usec=(++fed)*33333;
				xioctl(dec,VIDIOC_QBUF,&b,"dec QBUF out");
				if (nxt<0) eos=1; else pos=nxt;
			}
		}
		/* encoder finished reading a frame -> recycle that buffer to the decoder */
		if (pfd[1].revents & POLLOUT){
			struct v4l2_buffer e={0}; e.type=V4L2_BUF_TYPE_VIDEO_OUTPUT; e.memory=V4L2_MEMORY_DMABUF;
			if (xioctl(enc,VIDIOC_DQBUF,&e,"enc DQBUF out")==0){
				struct v4l2_buffer q={0}; q.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; q.memory=V4L2_MEMORY_MMAP; q.index=e.index;
				xioctl(dec,VIDIOC_QBUF,&q,"dec reQBUF cap");
			}
		}
		/* encoder produced an H264 AU */
		if (pfd[1].revents & POLLIN){
			struct v4l2_buffer e={0}; e.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; e.memory=V4L2_MEMORY_MMAP;
			if (xioctl(enc,VIDIOC_DQBUF,&e,"enc DQBUF cap")==0){
				if (e.bytesused>0){ encoded++; fwrite(ecb[e.index].p,1,e.bytesused,of); }
				xioctl(enc,VIDIOC_QBUF,&e,"enc reQBUF cap");
			}
		}
		if (eos && dq>0 && !(pfd[0].revents & POLLIN) && encoded>0 && it>50){
			/* heuristic drain end */
		}
	}

	fclose(of);
	printf("DONE: dec frames dq=%d, encoded AUs=%d -> %s\n", dq, encoded, outf);
	for (int i=0;i<ndc;i++) close(expfd[i]);
	close(dec); close(enc);
	return 0;
}
