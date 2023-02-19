import socket
import time
import requests
import json
import atexit
import sys
import psutil

usb_interface=psutil.net_if_addrs()["usb0"][0]
if usb_interface == None:
    print("GoPro not connected properly! No usb0 interface")
    sys.exit()

GOPRO_IP = usb_interface.address [:-1] + "1"
print("GoPro connected on "+GOPRO_IP)

START_URL = "http://"+GOPRO_IP+"/gp/gpWebcam/START?res=1080"
STOP_URL = "http://"+GOPRO_IP+"/gp/gpWebcam/STOP"

try:
    args = sys.argv[1].split(':')

    UDP_IP = args[0]
    UDP_PORT = int(args[1])
except:
    print("Parameter needed in form of <IP:PORT>")
    sys.exit()

def exit_handler():
    print('\nStopping Webcam...\n')
    requests.get(STOP_URL)

atexit.register(exit_handler)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind(("", 8554))



r = requests.get(START_URL)
r_json = json.loads(r.text)

if(r_json['error'] == 0 and r_json['status'] == 2):
    print("\nGoPro Webcam mode startet\n")
    print("Forward UDP stream to " + UDP_IP + ":" + str(UDP_PORT) + "\n")
    while True:
        data, addr = sock.recvfrom(2048)
        #print("received message:", data)
        sock.sendto(data, (UDP_IP, UDP_PORT))


    x = input()
    print ('Try using KeyboardInterrupt')

#vlc -vvv --network-caching=300 --sout-x264-preset=ultrafast --sout-x264-tune=zerolatency --sout-x264-vbv-bufsize 0 --sout-transcode-threads 4 --no-audio udp://@:8554
#import socket
#import struct
#import time
#import sys
#
## create socket
#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#
## bind socket to port 5887
#server_address = ('', 8554)
#print('starting up on {} port {}'.format(*server_address))
#sock.bind(server_address)
#
## connect to 192.168.178.20:5888
#destination = ('192.168.178.65', 8554)
#print('connecting to {} port {}'.format(*destination))
#sock.connect(destination)
#
## send RTP header
#sock.send(struct.pack('!BBHII', 0x80, 0x60, 0x00, 0x00, 0x00))
#
## send data
#while True:
#    data, address = sock.recvfrom(4096)
#    print('received {} bytes from {}'.format(len(data), address))
#    sock.send(data)