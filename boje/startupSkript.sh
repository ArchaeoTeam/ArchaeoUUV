#!/bin/bash

sleep 30
# Start mavproxy
sudo -H -u pi screen -d -m -S mavproxy.py /home/pi/.local/bin/mavproxy.py --out=udp:127.0.0.1:14550 > /home/pi/test.log
#/home/pi/.local/bin/mavproxy.py --out=udp:127.0.0.1:14550  --default-modules='output, param, calibration' > /home/pi/test.log &
printf "\nStarting Mavproxy...\n"
sleep 5

# start Webserver for sensordata
#sudo screen -d -m -S Sensor_server bash -c 'python3 /home/pi/ArchaeoUUV/BOJE/tempHum.py'
#printf "Starting Webserver...\n"

sleep 15

sudo -H -u pi screen -d -m -S GPS_calc bash -c 'python3 /home/pi/ArchaeoUUV/BOJE/GPS_calc.py'
printf "Stream GPS\n"
