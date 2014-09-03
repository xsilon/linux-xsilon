#! /bin/sh

# Make sure only root can run our script
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

usage()
{
cat << EOF
usage: $0 options

Populate the TFTP server with the linux and device tree binary for a given
board type. 

OPTIONS:
   -d      Override deploy directory that contains the linux and DTB images.
EOF
}

BOARD=han9250
while getopts “hm:d:” OPTION; do
	case $OPTION in
	h)
		usage
		exit 1
		;;
	d)
		DEPLOY_DIR=${OPTARG}
		;;
	?)
		usage
		exit
		;;
	esac
done

if [ -z ${BOARD} ]; then
	echo "You must set the board type"
	usage
	exit 1
fi

LINUX_BUILT_FILENAME=linux.bin.ub
LINUX_TFTP_FILENAME=linux.bin.ub-${BOARD}-microblazeel.bin
DEPLOY_DIR=arch/microblaze/boot/
TFTP_DIR=/var/lib/tftpboot

# Copy Linux binary with U-Boot header
echo "Copying ${DEPLOY_DIR}/${LINUX_BUILT_FILENAME} to TFTP dir"
cp ${DEPLOY_DIR}/${LINUX_BUILT_FILENAME} ${TFTP_DIR}/${LINUX_TFTP_FILENAME}

exit 0


