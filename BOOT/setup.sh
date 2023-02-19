#!/bin/bash
yes | sudo apt-get install build-essential python3-dev python3-smbus python3-pip
yes | pip install numpy
yes | pip3 install numpy
yes | sudo pip3 install adafruit-circuitpython-ads1x15
yes | pip3 install Adafruit-Blinka

#installing docker
mkdir docker_install
cd docker_install
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
docker info