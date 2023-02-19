#!/usr/bin/env bash

# exit when any command fails
set -e

# Check if the script is running as root
[[ $EUID != 0 ]] && echo "Script must run as root."  && exit 1


BUILD_TOOLS=(
    binutils
	gcc
    g++
    wget
    unzip
    make
	build-essential
	python-dev
	python3-dev
	python3-smbus
	python3-pip	
	python3-gpg
	libc-dev
	libevent-dev
	gpg
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


cd ..
rm master.zip

wget https://github.com/sarnold/RPi.GPIO/archive/refs/heads/master.zip
unzip master.zip
cd RPi.GPIO-master
python3 setup.py install


#apt-get -y remove ${BUILD_TOOLS[*]}
#apt-get -y autoremove
#apt-get -y clean
pip3 install appdirs
pip3 install aiohttp
#raspi-config nonint do_i2c 0
