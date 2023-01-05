#!/bin/bash
#~/companion/GPS_TEST.sh &
#x=$(echo $!)
#echo $x
#sleep 15
#sudo kill $x
while true; do
	#sleep 3
	
	sudo stty \
    -F /dev/ttyUSB0 \
    raw \
    115200 \
    cs8 \
    clocal \
    cs8 \
    -parenb \
    crtscts \
    -cstopb
	
	x=1

	
	#while [ "$line2" != "$GNZDA" ]
	#do	
	#		echo "Starte lesen"
	#	read -r line < /dev/ttyUSB0
	#	line2=${line:0:6}
	#	echo $line
	#		echo $line2
	#done
	echo "Setzte Zeit"
	exec 4</dev/ttyUSB0 # gps read-stream
	
	while [ $x -eq 1 ]
	do     
		read this_line;  
		if [[ $this_line =~ "GNZDA" ]]
		then
			echo "Gefunden"
			echo $this_line;
			day=${this_line:18:2}
			month=${this_line:21:2}
			year=${this_line:24:4}
			hour=${this_line:7:2}
			correction=1
			#hour=$hour
			#hour=$(($hour+$correction))
			minutes=${this_line:9:2}
			seconds=${this_line:11:2}
			date="$year-$month-$day $hour:$minutes:$seconds.000"
			echo $date
			sudo date --set="$date"
			sshpass -p companion ssh pi@192.168.2.2 "sudo date --set='$date'"
			x=0
		fi
	done <&4
	
	
	
	echo "Starte GPS"
	cat /dev/ttyUSB0 > /dev/udp/192.168.2.2/27000
	
	
done

