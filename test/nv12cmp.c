// SPDX-License-Identifier: GPL-2.0
/*
 * nv12cmp — pixel-level sanity for decoded NV12 dumps (KI-003 validation).
 *   nv12cmp A.nv12 [B.nv12] W H [maxframes]
 *
 * For each frame i (up to maxframes, default 16):
 *   - is A[i] all-zero (black/undecoded)?
 *   - if B given: luma MAD(A[i],B[i]) — small => same content (NV12 conv diff ok)
 * Prints how many of A's frames are non-zero and how many are distinct.
 * This exists because "bytesused!=0"/"ours=1" do NOT prove real pixels; only
 * comparing pixels (non-zero, distinct, close to a software reference) does.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
	if (argc < 4) {
		fprintf(stderr, "usage: %s A.nv12 [B.nv12] W H [maxframes]\n", argv[0]);
		return 2;
	}
	int has_b = (argc >= 5 && atoi(argv[3]) == 0) ? 0 : 0; /* placeholder */
	/* Parse: if argv[2] is a number, there is no B. */
	const char *fa = argv[1];
	const char *fb = NULL;
	int ai = 2;
	if (atoi(argv[2]) != 0) {            /* argv[2] is W -> no B */
		fb = NULL; ai = 2;
	} else {
		fb = argv[2]; ai = 3;
	}
	if (argc < ai + 2) { fprintf(stderr, "need W H\n"); return 2; }
	int W = atoi(argv[ai]), H = atoi(argv[ai + 1]);
	int maxf = (argc > ai + 2) ? atoi(argv[ai + 2]) : 16;
	(void)has_b;
	if (W <= 0 || H <= 0) { fprintf(stderr, "bad W/H\n"); return 2; }

	long ysz = (long)W * H;
	long fsz = ysz * 3 / 2;
	unsigned char *a = malloc(fsz), *b = fb ? malloc(fsz) : NULL;
	unsigned char *prev = malloc(fsz);
	if (!a || (fb && !b) || !prev) { fprintf(stderr, "oom\n"); return 2; }

	FILE *FA = fopen(fa, "rb"); if (!FA) { perror(fa); return 2; }

	/* Load all B (software ref) frames into memory for best-match search. */
	int nb = 0; unsigned char *B = NULL;
	if (fb) {
		FILE *FB = fopen(fb, "rb"); if (!FB) { perror(fb); return 2; }
		int cap = 64; B = malloc((size_t)cap * fsz);
		while (nb < 256) {
			if (nb >= cap) { cap *= 2; B = realloc(B, (size_t)cap * fsz); }
			if (fread(B + (size_t)nb * fsz, 1, fsz, FB) != (size_t)fsz) break;
			nb++;
		}
		fclose(FB);
	}

	int i, nonzero = 0, distinct = 0, have_prev = 0, aligned = 0;
	if (fb) printf("%-5s %-7s %-9s %-7s %-9s\n",
		       "Aframe", "Azero?", "MAD@same", "bestB", "minMAD");
	else printf("%-5s %-7s\n", "Aframe", "Azero?");
	for (i = 0; i < maxf; i++) {
		if (fread(a, 1, fsz, FA) != (size_t)fsz) break;
		long s = 0; for (long k = 0; k < ysz; k++) s += a[k];
		int azero = (s == 0);
		if (!azero) nonzero++;
		if (!have_prev || memcmp(a, prev, fsz) != 0) distinct++;
		memcpy(prev, a, fsz); have_prev = 1;

		if (fb && nb) {
			double mad_best = 1e18; int bestj = -1;
			for (int j = 0; j < nb; j++) {
				const unsigned char *bb = B + (size_t)j * fsz;
				long d = 0; for (long k = 0; k < ysz; k++) {
					int x = (int)a[k] - (int)bb[k]; d += x < 0 ? -x : x;
				}
				double mad = (double)d / ysz;
				if (mad < mad_best) { mad_best = mad; bestj = j; }
			}
			/* Fit A ~= g*B + o (least squares over luma) and report the
			 * residual MAD. A pure range/gain difference (limited vs full,
			 * studio vs PC) collapses to ~0 here => SAME content. A real
			 * spatial/content difference does not. */
			const unsigned char *bb = B + (size_t)bestj * fsz;
			double sa = 0, sb = 0, sbb = 0, sab = 0, n = ysz;
			for (long k = 0; k < ysz; k++) {
				double x = a[k], y = bb[k];
				sa += x; sb += y; sbb += y * y; sab += x * y;
			}
			double det = n * sbb - sb * sb;
			double g = det != 0 ? (n * sab - sa * sb) / det : 1.0;
			double o = (sa - g * sb) / n;
			double res = 0;
			for (long k = 0; k < ysz; k++) {
				double x = (double)a[k] - (g * bb[k] + o);
				res += x < 0 ? -x : x;
			}
			res /= ysz;
			if (res < 3.0) aligned++;
			printf("%-5d %-7s bestB=%-3d MAD=%-6.2f meanA=%-5.1f meanB=%-5.1f gain=%-5.3f res=%-5.2f\n",
			       i, azero ? "ZERO" : "ok", bestj, mad_best, sa / n, sb / n, g, res);
		} else {
			printf("%-5d %-7s\n", i, azero ? "ZERO" : "ok");
		}
	}
	printf("-- A:frames=%d nonzero=%d distinct=%d ; linfit-res<3 (SAME content)=%d/%d (B has %d)\n",
	       i, nonzero, distinct, aligned, i, nb);
	return 0;
}
