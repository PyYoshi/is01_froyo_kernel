#!/bin/sh
./mkbootfs root > ramdisk.img
./mkbootimg --kernel ../Image --ramdisk ramdisk.img --cmdline "console=ttyMSM2,115200n8 androidboot.hardware=qcom" --base 0x20000000 -o boot.img
ubinize -o ubi_boot.img -p 128KiB -m 2048 -O 256 ubi.cfg
rm ramdisk.img
rm boot.img
