#!/bin/sh

# Source compiler settings
source /opt/Xilinx/SDK/2014.2/settings64.sh /opt/Xilinx/SDK/2014.2

make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- -j8

