#!/bin/bash
x=`date '+%Y.%m.%d_%H:%M:%S_GPSRTK.log'`

while [ 1 ]
do
      echo "Set stty"
      ./sshlink.sh pi raspberry 192.168.2.115 stty -F /dev/ttyUSB0 115200
	  echo "Send Data"
	  ./sshlink.sh pi raspberry 192.168.2.115 cat /dev/ttyUSB0 | grep GGA > /dev/udp/192.168.2.3/28000  
done
