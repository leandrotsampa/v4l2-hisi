/* Minimal U-Boot env editor: validate CRC, replace one var, rewrite.
 * Env layout (non-redundant): [u32 CRC32 LE][data ... \0\0 ... 0x00 pad].
 * CRC is over the whole data area (ENV_SIZE-4 bytes). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define ENV_SIZE 0x10000u
#define DATA_OFF 4u
#define DATA_LEN (ENV_SIZE - DATA_OFF)

static uint32_t crc32_calc(const unsigned char *p, size_t n)
{
	uint32_t c = 0xFFFFFFFFu;
	size_t i; int k;
	for (i = 0; i < n; i++) {
		c ^= p[i];
		for (k = 0; k < 8; k++)
			c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int)(c & 1)));
	}
	return c ^ 0xFFFFFFFFu;
}

int main(int argc, char **argv)
{
	/* argv: in.bin [out.bin VARNAME NEWVALUE] */
	FILE *f = fopen(argv[1], "rb");
	static unsigned char buf[ENV_SIZE];
	if (!f || fread(buf, 1, ENV_SIZE, f) != ENV_SIZE) { fprintf(stderr, "read fail\n"); return 2; }
	fclose(f);

	uint32_t stored = buf[0] | (buf[1]<<8) | (buf[2]<<16) | ((uint32_t)buf[3]<<24);
	uint32_t calc = crc32_calc(buf + DATA_OFF, DATA_LEN);
	printf("stored CRC=0x%08x  calc CRC=0x%08x  %s\n",
	       stored, calc, stored == calc ? "MATCH" : "*** MISMATCH ***");
	if (stored != calc) { fprintf(stderr, "CRC mismatch — abort\n"); return 3; }

	/* dump current value of the target var if given */
	if (argc >= 3) {
		const char *var = argv[3];
		size_t vlen = strlen(var);
		unsigned char *d = buf + DATA_OFF, *end = buf + ENV_SIZE, *p = d;
		while (p < end && *p) {
			char *eq = memchr(p, '=', strlen((char*)p));
			if (eq && (size_t)(eq - (char*)p) == vlen && !memcmp(p, var, vlen))
				printf("OLD %s\n", (char*)p);
			p += strlen((char*)p) + 1;
		}
	}

	if (argc < 5) return 0;   /* validate/dump only */

	/* rebuild data, replacing VARNAME with VARNAME=NEWVALUE */
	const char *var = argv[3], *newval = argv[4];
	size_t vlen = strlen(var);
	static unsigned char out[ENV_SIZE];
	memset(out, 0, ENV_SIZE);
	unsigned char *w = out + DATA_OFF;
	unsigned char *d = buf + DATA_OFF, *end = buf + ENV_SIZE, *p = d;
	int replaced = 0;
	while (p < end && *p) {
		char *eq = memchr(p, '=', strlen((char*)p));
		size_t entlen = strlen((char*)p);
		if (eq && (size_t)(eq - (char*)p) == vlen && !memcmp(p, var, vlen)) {
			int n = sprintf((char*)w, "%s=%s", var, newval);
			w += n + 1; replaced = 1;
			printf("NEW %s=%s\n", var, newval);
		} else {
			memcpy(w, p, entlen + 1); w += entlen + 1;
		}
		p += entlen + 1;
	}
	*w++ = 0;   /* final terminator (empty entry) */
	if (!replaced) { fprintf(stderr, "var %s not found — abort\n", var); return 4; }

	uint32_t ncrc = crc32_calc(out + DATA_OFF, DATA_LEN);
	out[0] = ncrc & 0xff; out[1] = (ncrc>>8)&0xff; out[2] = (ncrc>>16)&0xff; out[3] = (ncrc>>24)&0xff;
	printf("new CRC=0x%08x\n", ncrc);

	FILE *o = fopen(argv[2], "wb");
	if (!o || fwrite(out, 1, ENV_SIZE, o) != ENV_SIZE) { fprintf(stderr, "write fail\n"); return 5; }
	fclose(o);
	printf("wrote %s\n", argv[2]);
	return 0;
}
