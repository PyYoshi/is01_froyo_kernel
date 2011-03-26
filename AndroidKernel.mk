#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/Image

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

ifeq ($(TARGET_BUILD_VARIANT),user)
define format_kernel_config_engineering
endef
else
define format_kernel_config_engineering
	awk -F '[ =]' 'function no(a){print a"=y"} function yes(a){print "\# "a" is not set"} {if($$1~/^CONFIG_/)name=$$1;else if($$2~/CONFIG_/)name=$$2;else{print; next};if(name~/^CONFIG_DEVK?MEM$$/)no(name);else if(name=="CONFIG_ANDROID_ENGINEERING")no(name);else if(name=="CONFIG_SECURITY_DECKARD")yes(name);else print}' $(1) > $(KERNEL_OUT)/tmp
	rm $(1)
	cp $(KERNEL_OUT)/tmp $(1)
endef
endif

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)
	$(call format_kernel_config_engineering, $(KERNEL_CONFIG))

$(TARGET_PREBUILT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	cp $(KERNEL_OUT)/.config kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

endif
