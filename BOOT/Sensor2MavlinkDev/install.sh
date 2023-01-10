#!/usr/bin/env bash

# exit when any command fails
set -e

# Check if the script is running as root
[[ $EUID != 0 ]] && echo "Script must run as root."  && exit 1


BUILD_TOOLS=(
    binutils
    g++
    wget
    unzip
    make
	build-essential
	python-dev
	python-smbus
	python-pip
)

BUILD_LIBS=(
)

# Install necessary dependencies
apt update

apt -y install ${BUILD_TOOLS[*]}
apt -y install ${BUILD_LIBS[*]}

wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make -j$(nproc)
make install

apt -y remove ${BUILD_TOOLS[*]}
apt -y autoremove
apt -y clean
sudo raspi-config nonint do_i2c 0
