#===============================================================================
# Hisilicon V4L2 codec driver (mem2mem video decoder + encoder) -> hi_v4l2.ko
#
# In-tree under hisilicon-kernel/drivers/msp/drv/, same pattern as dvb-hisi:
# include base.mak, declare MOD_NAME + the object list, and let the parent
# drivers/msp/drv/Makefile pick the dir up via `objects += v4l2-hisi`.
#===============================================================================
ifeq ($(CFG_HI_EXPORT_FLAG),)
    ifneq ($(KERNELRELEASE),)
		KERNEL_DIR := $(srctree)
		SDK_DIR := $(shell cd $(KERNEL_DIR)/../../.. && /bin/pwd)
    else
		SDK_DIR := $(shell cd $(CURDIR)/../../../.. && /bin/pwd)
    endif

    ifeq ($(wildcard $(SDK_DIR)/base.mak),)
        ifneq ($(srctree),)
            KERNEL_DIR := $(srctree)
            SDK_DIR := $(shell cd $(KERNEL_DIR) && /bin/pwd)
        else
            SDK_DIR := $(shell cd $(CURDIR)/.. && /bin/pwd)
        endif
    endif

    include $(SDK_DIR)/base.mak
endif

#===============================================================================
# include paths
#===============================================================================
EXTRA_CFLAGS += $(CFG_HI_KMOD_CFLAGS)
EXTRA_CFLAGS += $(CFG_HI_BOARD_CONFIGS)
EXTRA_CFLAGS += -I$(COMMON_UNF_INCLUDE)                     \
                -I$(COMMON_API_INCLUDE)                     \
                -I$(COMMON_DRV_INCLUDE)                     \
                -I$(COMMON_DRV_INCLUDE)/$(CFG_HI_CHIP_TYPE) \
                -I$(MSP_UNF_INCLUDE)                        \
                -I$(MSP_API_INCLUDE)                        \
                -I$(MSP_DRV_INCLUDE)                        \
                -I$(COMMON_DIR)/drv/mmz                     \
                -I$(MSP_DIR)/drv/vdec/vdec_v2.0             \
                -I$(MSP_DIR)/drv/vfmw/vfmw_v5.0             \
                -I$(CURDIR)

EXTRA_CFLAGS += -DHI_CHIP_TYPE=\"$(CFG_HI_CHIP_TYPE)\"
# Dynamic decode capability probe via HI_DRV_VDEC_GetCap (vfmw.h reachable above).
EXTRA_CFLAGS += -DHI_V4L2_USE_VDEC_GETCAP

# Zero-copy Fase C (KI-003) isolated probe (Passo 2). Gates hi_v4l2_zc_test.c and
# the hi_vdec_hal_zc_* seam. REMOVE this flag (and the file) after Passo 3 lands.
EXTRA_CFLAGS += -DHI_V4L2_ZC_SELFTEST

# Resolve the exported HI_DRV_VDEC_* symbols (aggregated msp/common symvers).
KBUILD_EXTRA_SYMBOLS += $(COMMON_DIR)/drv/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(MSP_DIR)/drv/Module.symvers

#===============================================================================
# module
#===============================================================================
MOD_NAME := hi_v4l2

# Built-in when CFG_HI_MSP_BUILDIN=y (HI_DRV_BUILDTYPE=y), like dvb-hisi.
# Works at boot via initcall ordering: hi_v4l2_core.c registers with
# late_initcall_sync (level 7s), which runs AFTER the msp boot init
# (hi_init.c HI_DRV_LoadModules @ late_initcall, level 7) that does the
# VFMW/VDEC/VPSS *_DRV_ModInit. So when our decoder probe opens the VDEC, the
# VFMW/VPSS functions it needs are already registered. We only HI_DRV_VDEC_Open
# (not Init/DeInit -- hi_init owns those), so we coexist with AVServer/AVPLAY.
# Deps built-in via the VIDEO_HISI_V4L2_DEPS Kconfig stub. Build w/ CONFIG_MSP=y.
obj-$(HI_DRV_BUILDTYPE) += $(MOD_NAME).o

$(MOD_NAME)-y := hi_v4l2_core.o \
                 hi_v4l2_dec.o \
                 hi_v4l2_enc.o \
                 hi_v4l2_fmt.o \
                 hi_vdec_hal.o \
                 hi_venc_hal.o \
                 hi_v4l2_zc_test.o

REMOVED_FILES := "*.o" "*.ko" "*.order" "*.symvers" "*.mod" "*.mod.*" "*.cmd" ".tmp_versions" "modules.builtin"

#===============================================================================
# rules
#===============================================================================
.PHONY: all clean install uninstall

all:
	$(AT)make -C $(LINUX_DIR) ARCH=$(CFG_HI_CPU_ARCH) CROSS_COMPILE=$(HI_KERNEL_TOOLCHAINS_NAME)- M=$(CURDIR) modules

debug:
	$(AT)make -C $(LINUX_DIR) CFLAGS=-g ARCH=$(CFG_HI_CPU_ARCH) CROSS_COMPILE=$(HI_KERNEL_TOOLCHAINS_NAME)- M=$(CURDIR) modules

strip: all
	$(HI_KERNEL_TOOLCHAINS_NAME)-strip --strip-debug $(MOD_NAME).ko
	$(HI_KERNEL_TOOLCHAINS_NAME)-strip --strip-unneeded $(MOD_NAME).ko

clean:
	$(AT)find ./ -name "*.d" $(foreach file, $(REMOVED_FILES), -o -name $(file)) | xargs rm -rf

install: all
	$(AT)cp -f $(CURDIR)/$(MOD_NAME).ko $(HI_MODULE_DIR)/

uninstall:
	$(AT)rm -rf $(HI_MODULE_DIR)/$(MOD_NAME).ko
