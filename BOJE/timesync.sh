#/bin/bash

ssh pi@192.168.2.2 -t "sudo date -s '@`date +%s`'"
