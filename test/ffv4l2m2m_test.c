// SPDX-License-Identifier: GPL-2.0
/*
 * ffv4l2m2m_test — prove that the board's *system* libavcodec (the very one Kodi
 * links, libavcodec.so.58) can decode through ffmpeg's V4L2 M2M decoder, which
 * drives our hi_v4l2 /dev/video0. This is the EXACT path Kodi's
 * CDVDVideoCodecFFmpeg would take once it selects "<codec>_v4l2m2m".
 *
 * Build natively on the target:
 *   gcc ffv4l2m2m_test.c -o ffv4l2m2m_test -lavformat -lavcodec -lavutil
 * Run:
 *   ./ffv4l2m2m_test <file> [decoder_name]
 *   # e.g. ./ffv4l2m2m_test bbb.webm vp8_v4l2m2m   (HW, our driver)
 *   #      ./ffv4l2m2m_test bbb.webm vp8           (software, baseline)
 */
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
	const char *file, *decname;
	AVFormatContext *fmt = NULL;
	AVCodecParameters *par;
	AVCodec *dec;
	AVCodecContext *ctx;
	AVFrame *frame;
	AVPacket pkt;
	int vstream = -1, frames = 0, i;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <file> [decoder_name]\n", argv[0]);
		return 1;
	}
	file = argv[1];
	decname = (argc > 2) ? argv[2] : "vp8_v4l2m2m";

	if (avformat_open_input(&fmt, file, NULL, NULL) < 0) {
		fprintf(stderr, "open input failed: %s\n", file);
		return 1;
	}
	avformat_find_stream_info(fmt, NULL);

	for (i = 0; i < (int)fmt->nb_streams; i++)
		if (fmt->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
			vstream = i;
			break;
		}
	if (vstream < 0) {
		fprintf(stderr, "no video stream\n");
		return 1;
	}
	par = fmt->streams[vstream]->codecpar;

	/* "auto" = the default path Kodi takes (avcodec_find_decoder by id);
	 * use it to prove the LD_PRELOAD shim redirects to *_v4l2m2m. */
	if (!strcmp(decname, "auto"))
		dec = avcodec_find_decoder(par->codec_id);
	else
		dec = avcodec_find_decoder_by_name(decname);
	if (!dec) {
		fprintf(stderr, "decoder '%s' NOT FOUND in this libavcodec\n", decname);
		return 2;
	}
	printf("decoder: %s (%s)\n", dec->name,
	       dec->long_name ? dec->long_name : "");

	ctx = avcodec_alloc_context3(dec);
	avcodec_parameters_to_context(ctx, par);
	if (avcodec_open2(ctx, dec, NULL) < 0) {
		fprintf(stderr, "avcodec_open2('%s') FAILED\n", decname);
		return 3;
	}

	frame = av_frame_alloc();
	while (av_read_frame(fmt, &pkt) >= 0) {
		if (pkt.stream_index == vstream &&
		    avcodec_send_packet(ctx, &pkt) == 0) {
			while (avcodec_receive_frame(ctx, frame) == 0) {
				frames++;
				if (frames == 1 || frames % 30 == 0)
					printf("frame %d: %dx%d fmt=%s\n", frames,
					       frame->width, frame->height,
					       av_get_pix_fmt_name(frame->format));
			}
		}
		av_packet_unref(&pkt);
	}
	avcodec_send_packet(ctx, NULL);                 /* flush */
	while (avcodec_receive_frame(ctx, frame) == 0)
		frames++;

	printf("TOTAL frames decoded via %s: %d\n", decname, frames);
	return frames > 0 ? 0 : 4;
}
