// SPDX-License-Identifier: GPL-2.0
/*
 * Hisilicon V4L2 driver - module entry. Registers both the mem2mem video
 * decoder (/dev/videoN) and encoder (/dev/videoM) over the proven HI_DRV_VDEC_*
 * / HI_DRV_VENC_* kernel layer.
 */
#include <linux/module.h>
#include <linux/init.h>

#include "hi_v4l2_drv.h"

static int __init hi_v4l2_init(void)
{
	int ret;

	ret = hi_dec_register();
	if (ret) {
		pr_err("hi-v4l2: decoder register failed (%d)\n", ret);
		return ret;
	}

	ret = hi_enc_register();
	if (ret) {
		pr_err("hi-v4l2: encoder register failed (%d)\n", ret);
		hi_dec_unregister();
		return ret;
	}

#ifdef HI_V4L2_ZC_SELFTEST
	hi_zc_test_register();		/* isolated zero-copy probe (KI-003) */
#endif

	pr_info("hi-v4l2: decoder + encoder registered\n");
	return 0;
}

static void __exit hi_v4l2_exit(void)
{
#ifdef HI_V4L2_ZC_SELFTEST
	hi_zc_test_unregister();
#endif
	hi_enc_unregister();
	hi_dec_unregister();
}

/*
 * Same idiom as dvb-hisi (drivers/msp/drv/dvb-hisi): late_initcall_sync works
 * for both build kinds -- under CONFIG MODULE all *_initcall macros collapse to
 * module_init (runs at insmod, stack already up); built-in it is initcall level
 * 7s, which runs AFTER the msp boot init (hi_init.c HI_DRV_LoadModules @
 * late_initcall, level 7) that does VFMW/VDEC/VPSS *_DRV_ModInit. We must run
 * after it: the decoder probe opens the VDEC (HI_DRV_VDEC_Open -> GetFunction
 * of VFMW/VPSS) and probes caps (HI_DRV_VDEC_GetCap), which need those
 * subsystems already registered. Plain late_initcall (level 7) raced
 * HI_DRV_LoadModules by link order and lost (VDEC_DRV_Open -> 0xffffffff).
 */
late_initcall_sync(hi_v4l2_init);
module_exit(hi_v4l2_exit);

MODULE_DESCRIPTION("Hisilicon V4L2 mem2mem video codec (decoder + encoder)");
MODULE_AUTHOR("HiSTBLinux V4L2 migration");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2.0");
