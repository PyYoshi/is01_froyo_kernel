ARCH := arm
TMP_DIR := $(PWD)
SDIO_DRIVER := mmc
KDIR := $(ANDROID_ROOT)/Linux
CROSS_COMPILE := $(ANDROID_ROOT)/prebuilt/linux-x86/toolchain/arm-eabi-4.2.1/bin/arm-eabi-
EXTRA_DRV_CFLAGS := -DUNIFI_NET_NAME=\"wlan\" -DANDROID_BUILD
export EXTRA_DRV_CFLAGS

include config.generic.mk
