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

module_init(hi_v4l2_init);
module_exit(hi_v4l2_exit);

MODULE_DESCRIPTION("Hisilicon V4L2 mem2mem video codec (decoder + encoder)");
MODULE_AUTHOR("HiSTBLinux V4L2 migration");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2.0");
