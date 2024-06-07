# Copyright (c) 2015 OpenWrt.org
#
include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=mediatek
BOARDNAME:=MediaTek Ralink ARM
SUBTARGETS:=mt7622 mt7623 mt7629 mt7986 mt7981 mt7988
FEATURES:=squashfs nand ramdisk fpu

KERNEL_PATCHVER:=5.4
KERNEL_TESTING_PATCHVER:=5.4

include $(INCLUDE_DIR)/target.mk
DEFAULT_PACKAGES += \
	kmod-leds-gpio kmod-gpio-button-hotplug \
	autocore-arm luci-app-cpufreq luci-app-eqos-mtk \
	luci-app-turboacc-mtk luci-app-upnp mii_mgr mtkhqos_util luci-app-wrtbwmon \
	mtk-smp mii_mgr switch regs luci-app-usbmodem swconfig luci-app-cpufreq kmod-sfp

$(eval $(call BuildTarget))
