/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Internal registration hooks for the single hi_v4l2 module, which exposes
 * both the decoder (/dev/videoN) and the encoder (/dev/videoM) devices.
 */
#ifndef __HI_V4L2_DRV_H__
#define __HI_V4L2_DRV_H__

int  hi_dec_register(void);
void hi_dec_unregister(void);

int  hi_enc_register(void);
void hi_enc_unregister(void);

/* CMA-backed device of the decoder (vb2-dma-contig parent). */
struct device;
struct device *hi_dec_dma_dev(void);

#ifdef HI_V4L2_ZC_SELFTEST
/* Zero-copy Fase C (KI-003) isolated probe (debugfs-triggered). */
int  hi_zc_test_register(void);
void hi_zc_test_unregister(void);
#endif

#endif /* __HI_V4L2_DRV_H__ */
