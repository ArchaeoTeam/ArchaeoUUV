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
    git
)

BUILD_LIBS=(
)

# Install necessary dependencies
apt update

apt -y install ${BUILD_TOOLS[*]}
apt -y install ${BUILD_LIBS[*]}

git clone https://github.com/ArchaeoTeam/ArchaeoUUV.git
pip3 install -r ArchaeoUUV/BOOT/requirements.txt

apt -y install 

apt -y remove ${BUILD_TOOLS[*]}
apt -y autoremove
apt -y clean
