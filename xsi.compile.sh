#!/bin/sh

# Emit a useful diagnostic if something fails:
bb_exit_handler() {
    ret=$?
    case $ret in
    0)  ;;
    *)  case $BASH_VERSION in
        "")   echo "WARNING: exit code $ret from a shell command.";;
        *)    echo "WARNING: ${BASH_SOURCE[0]}:${BASH_LINENO[0]} exit $ret from
  "$BASH_COMMAND"";;
        esac
        exit $ret
    esac
}
trap 'bb_exit_handler' 0
set -e
export prefix="/usr"
export STRIP="microblazeel-oe-linux-strip"
export localstatedir="/var"
export BUILD_CC="gcc "
export libexecdir="/usr/lib/linux-xlnx"
export datadir="/usr/share"
export BUILD_CXX="g++ "
export LD="microblazeel-oe-linux-ld --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel "
export ARCH="microblaze"
export bindir="/usr/bin"
export TARGET_CXXFLAGS=" -O -fno-omit-frame-pointer -g -feliminate-unused-debug-types -pipe"
export BUILD_LDFLAGS="-L/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/lib -L/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/lib -Wl,-rpath-link,/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/lib -Wl,-rpath-link,/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/lib -Wl,-rpath,/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/lib -Wl,-rpath,/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/lib -Wl,-O1"
export STRINGS="microblazeel-oe-linux-strings"
export BUILD_LD="ld "
export oldincludedir="/usr/include"
export BUILD_CCLD="gcc "
export CC="microblazeel-oe-linux-gcc   -mlittle-endian    -mcpu=v8.50.a        -mxl-barrel-shift -mno-xl-soft-mul -mxl-multiply-high  -mno-xl-soft-div  -mxl-pattern-compare -mxl-reorder --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel"
export CMDLINE_CONSOLE="console=ttyS0"
export CPPFLAGS=""
export RANLIB="microblazeel-oe-linux-ranlib"
export CXX="microblazeel-oe-linux-g++   -mlittle-endian    -mcpu=v8.50.a        -mxl-barrel-shift -mno-xl-soft-mul -mxl-multiply-high  -mno-xl-soft-div  -mxl-pattern-compare -mxl-reorder --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel"
export OS="linux"
export BUILD_NM="nm"
export mandir="/usr/share/man"
export BUILD_RANLIB="ranlib"
export BUILD_AR="ar"
export PKG_CONFIG_SYSROOT_DIR="/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel"
export OBJCOPY="microblazeel-oe-linux-objcopy"
export servicedir="/srv"
export base_prefix=""
export LC_ALL="C"
export PKG_CONFIG_DIR="/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel/usr/lib/pkgconfig"
export sysconfdir="/etc"
export CCLD="microblazeel-oe-linux-gcc   -mlittle-endian    -mcpu=v8.50.a        -mxl-barrel-shift -mno-xl-soft-mul -mxl-multiply-high  -mno-xl-soft-div  -mxl-pattern-compare -mxl-reorder --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel"
export includedir="/usr/include"
export PKG_CONFIG_LIBDIR="/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel/usr/lib/pkgconfig"
export sbindir="/usr/sbin"
export CFLAGS=" -O -fno-omit-frame-pointer -g -feliminate-unused-debug-types -pipe"
export TARGET_LDFLAGS="-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed"
export PSEUDO_DISABLED="1"
export PKG_CONFIG_DISABLE_UNINSTALLED="yes"
export libdir="/usr/lib"
export CXXFLAGS=" -O -fno-omit-frame-pointer -g -feliminate-unused-debug-types -pipe -fvisibility-inlines-hidden"
export USER="martin"
export LDFLAGS="-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed"
export MAKE="make"
unset TARGET_ARCH
export CCACHE_DIR="/home/martin"
export BUILD_CPP="cpp "
export BUILD_CFLAGS="-isystem/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/include -O2 -pipe"
export docdir="/usr/share/doc"
export infodir="/usr/share/info"
export TERM="xterm"
export base_sbindir="/sbin"
export systemd_unitdir="/lib/systemd"
export PKG_CONFIG_PATH="/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel/usr/lib/pkgconfig:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel/usr/share/pkgconfig"
export base_bindir="/bin"
export AS="microblazeel-oe-linux-as "
export AR="microblazeel-oe-linux-ar"
export BUILD_CPPFLAGS="-isystem/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/include"
export CPP="microblazeel-oe-linux-gcc -E --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel   -mlittle-endian    -mcpu=v8.50.a        -mxl-barrel-shift -mno-xl-soft-mul -mxl-multiply-high  -mno-xl-soft-div  -mxl-pattern-compare -mxl-reorder"
export BUILD_CXXFLAGS="-isystem/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/include -O2 -pipe"
export CCACHE_DISABLE="1"
export FC="microblazeel-oe-linux-gfortran   -mlittle-endian    -mcpu=v8.50.a        -mxl-barrel-shift -mno-xl-soft-mul -mxl-multiply-high  -mno-xl-soft-div  -mxl-pattern-compare -mxl-reorder --sysroot=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel"
export HOME="/home/martin"
export BUILD_FC="gfortran "
export TARGET_CPPFLAGS=""
export PATCH_GET="0"
export exec_prefix="/usr"
export NM="microblazeel-oe-linux-nm"
export base_libdir="/lib"
export LOGNAME="martin"
export PATH="/home/martin/xsi_workspace/poky/scripts:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/bin/microblazeel-v8.50-bs-cmp-re-mh-div-oe-linux:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/han9250-microblazeel/usr/bin/crossscripts:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/sbin:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/usr/bin:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/sbin:/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/sysroots/x86_64-linux/bin:/home/martin/xsi_workspace/poky/scripts:/home/martin/xsi_workspace/poky/bitbake/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games"
export TARGET_CFLAGS=" -O -fno-omit-frame-pointer -g -feliminate-unused-debug-types -pipe"
unset MACHINE
export BUILD_AS="as "
export sharedstatedir="/com"
export OBJDUMP="microblazeel-oe-linux-objdump"
export CROSS_COMPILE="microblazeel-oe-linux-"
unset DISTRO
export nonarch_base_libdir="/lib"
export PSEUDO_UNLOAD="1"
export SHELL="/bin/bash"
export UBOOT_ARCH="microblaze"
export BUILD_STRIP="strip"
do_compile() {
    kernel_do_compile

}

kernel_do_compile() {
	unset CFLAGS CPPFLAGS CXXFLAGS LDFLAGS MACHINE
	# The $use_alternate_initrd is only set from
	# do_bundle_initramfs() This variable is specifically for the
	# case where we are making a second pass at the kernel
	# compilation and we want to force the kernel build to use a
	# different initramfs image.  The way to do that in the kernel
	# is to specify:
	# make ...args... CONFIG_INITRAMFS_SOURCE=some_other_initramfs.cpio
	if [ "$use_alternate_initrd" = "" ] && [ "" != "" ] ; then
		# The old style way of copying an prebuilt image and building it
		# is turned on via INTIRAMFS_TASK != ""
		copy_initramfs
		use_alternate_initrd=CONFIG_INITRAMFS_SOURCE=/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.cpio
	fi
	oe_runmake linux.bin.ub  CC="microblazeel-oe-linux-gcc " LD="microblazeel-oe-linux-ld.bfd "  $use_alternate_initrd
	if test "linux.bin.ub.gz" = "linux.bin.ub"; then
		gzip -9c < "linux.bin.ub" > "arch/microblaze/boot/linux.bin.ub"
	fi

}

oe_runmake() {
	oe_runmake_call "$@" || die "oe_runmake failed"

}

copy_initramfs() {
	echo "Copying initramfs into ./usr ..."
	# In case the directory is not created yet from the first pass compile:
	mkdir -p /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr
	# Find and use the first initramfs image archive type we find
	rm -f /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.cpio
	for img in cpio.gz cpio.lz4 cpio.lzo cpio.lzma cpio.xz; do
		if [ -e "/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/deploy/images/han9250-microblazeel/-han9250-microblazeel.$img" ]; then
			cp /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/deploy/images/han9250-microblazeel/-han9250-microblazeel.$img /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/.
			case $img in
			*gz)
				echo "gzip decompressing image"
				gunzip -f /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.$img
				break
				;;
			*lz4)
				echo "lz4 decompressing image"
				lz4 -df /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.$img
				break
				;;
			*lzo)
				echo "lzo decompressing image"
				lzop -df /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.$img
				break
				;;
			*lzma)
				echo "lzma decompressing image"
				lzma -df /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.$img
				break
				;;
			*xz)
				echo "xz decompressing image"
				xz -df /home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git/usr/-han9250-microblazeel.$img
				break
				;;
			esac
		fi
	done
	echo "Finished copy of initramfs into ./usr"

}

die() {
	bbfatal "$*"

}

oe_runmake_call() {
	bbnote make -j 8  "$@"
	make -j 8  "$@"
	make -j 8 modules

}

bbfatal() {
	echo "ERROR: $*"
	exit 1

}

bbnote() {
	echo "NOTE: $*"

}

#cd '/home/martin/xsi_workspace/poky/yocto-build/han9250/debug/brian-han9250-microblazeel-tmp-eglibc/work/han9250_microblazeel-oe-linux/linux-xlnx/3.16-xilinx+gitAUTOINC-r0/git'
do_compile

# cleanup
ret=$?
trap '' 0
exit $?
