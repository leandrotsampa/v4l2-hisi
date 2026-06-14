/* Minimal libhicodec decoder test: H264 ES file -> NV21 frames file.
 * Confirms whether the userspace AVPLAY + virtual VO window path produces
 * LINEAR NV21 on this box (reference for the V4L2 VPSS-detile work).
 * Build (host, cross): arm-linux-gnueabihf-gcc dectest.c -o dectest \
 *   -I<hicodec.h dir> -L<libhicodec.so dir> -lhicodec \
 *   -Wl,--unresolved-symbols=ignore-in-shared-libs
 * Run (target): ./dectest /opt/big.264 /tmp/dec.nv21 20
 */
#include "hicodec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* HI_SYS_Init (source/common): registers modules incl. the VO Dolby-lib status.
 * Without it, vl_video_decoder_init -> HI_MPI_WIN_Init fails on
 * HI_MPI_WIN_SetDolbyLibStatus. Mirrors the SDK samples (showiframe etc.). */
extern int HI_SYS_Init(void);

int main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s in.264 out.nv21 [maxframes]\n", argv[0]);
		return 1;
	}
	int maxf = (argc > 3) ? atoi(argv[3]) : 20;
	SHICODEC codec;
	memset(&codec, 0, sizeof(codec));

	if (HI_SYS_Init() != 0)
		fprintf(stderr, "warn: HI_SYS_Init nonzero (continuing)\n");

	if (!vl_video_decoder_init(CODEC_ID_H264, &codec)) {
		fprintf(stderr, "vl_video_decoder_init FAIL\n");
		return 1;
	}
	fprintf(stderr, "CKPT: decoder_init OK\n");

	FILE *fi = fopen(argv[1], "rb");
	FILE *fo = fopen(argv[2], "wb");
	if (!fi || !fo) { perror("fopen"); return 1; }

	fseek(fi, 0, SEEK_END);
	long sz = ftell(fi);
	fseek(fi, 0, SEEK_SET);
	unsigned char *es = malloc(sz);
	if (fread(es, 1, sz, fi) != (size_t)sz) { fprintf(stderr, "read fail\n"); return 1; }

	unsigned char *framebuf = malloc(3840 * 2160 * 3 / 2);
	long off = 0, pts = 0;
	int frames = 0;

	while (off < sz && frames < maxf) {
		int chunk = (sz - off > 65536) ? 65536 : (int)(sz - off);
		fprintf(stderr, "CKPT: input off=%ld chunk=%d\n", off, chunk);
		vl_video_decoder_input(&codec, es + off, chunk, pts);
		off += chunk;
		pts += 40000;

		int osz;
		uint8_t *out = framebuf;
		int64_t opts = 0;
		fprintf(stderr, "CKPT: draining\n");
		while ((osz = vl_video_decoder_output(&codec, &out, &opts)) > 0
		       && frames < maxf) {
			fwrite(framebuf, 1, osz, fo);
			frames++;
			fprintf(stderr, "frame %d size=%d\n", frames, osz);
			out = framebuf;
		}
	}

	fprintf(stderr, "TOTAL frames=%d\n", frames);
	vl_video_decoder_destroy(&codec);
	fclose(fi);
	fclose(fo);
	free(es);
	free(framebuf);
	return 0;
}
