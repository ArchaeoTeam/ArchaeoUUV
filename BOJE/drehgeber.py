import spidev #import the SPI library for RB Pi 4 B board
import time #import the Timing library for RB Pi 4 B board
from urllib.parse import urlparse
import requests
import threading
import subprocess
import os
import socket

AMT22_NOP = 0 #command to read the position of the encoder
#AMT22_NOP = hex("00A00000")
AMT22_NOP = 0x00
AMT22_RESET = 0x60
AMT22_ZERO = 0x70
AMT22_READ_TURNS = 0xA0

NEWLINE = 0x0A
TAB = 0x09
spi = spidev.SpiDev() #create the spi object
spi.open(0, 0) #SPI port 0, CS 0
speed_hz=500000 #setting the speed in hz
delay_us=3 #setting the delay in microseconds

turns = 0
rotation = 0
initialrotation = 0
result = []

alt_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/AHRS2/message/altitude"
compass_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/VFR_HUD/message/heading"
depth = 0
compass = 0

def update_boot_values():
   while True:
      global depth
      global compass
      depth = requests.get(alt_url).text
      compass = requests.get(compass_url).text

def update_encoder_values():
   while True:
      global turns
      global rotation
      global result

      result=spi.xfer2([AMT22_NOP, AMT22_READ_TURNS, AMT22_NOP, AMT22_NOP],speed_hz,delay_us)

      rotation=16383-((result[0] & 0b111111) << 8) + result[1] # 0 - 16383 Pro umdrehung
      if initialrotation == 0:
         initialrotation = rotation
      
      turns=255-result[3]

      #lenght=(((calibturns-turns)*16383)-rotation+calibtotat)/370
      #print(lenght)

def rec_Boje():
    while True:
        #s.system("sudo stty -F /dev/ttyAMA0 115200")
        #date = subprocess.run(["date '+%Y.%m.%d_%H:%M:%S_GPSBOJE.log'"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        #print("date")
        #ime.sleep(1.1)
        #s.system("sudo cat /dev/ttyAMA0 | tee /home/pi/NMEA/GPSBOJE.log | grep GNGGA")
        time.sleep(1)
        
def send_RTK():
    while True:
        time.sleep(5)
        os.system("./sendRTK.sh")
        time.sleep(1)
        
def rec_RTK():
    UDP_IP = "192.168.2.3"
    UDP_PORT = 28000
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        print("received message: %s" % data)
    
        
#start depth & compass thread
thread_boot = threading.Thread(target=update_boot_values, args=(), daemon=True)
thread_boot.start()

#start encoder thread
thread_encoder = threading.Thread(target=update_encoder_values, args=(), daemon=True)
thread_encoder.start()

#thread_send_RTK = threading.Thread(target=send_RTK, args=(), daemon=True)
#thread_send_RTK.start()
#thread_rec_RTK = threading.Thread(target=rec_RTK, args=(), daemon=True)
#thread_rec_RTK.start()

while True:
   os.system("clear")
   #Print boot data
   print("Tiefe")
   print(depth)
   print("Kompass")
   print(compass)

   #Print Turns
   fturn=turns+(rotation-initialrotation/16383)
   print("Turns: " + str(fturn))

   #Convert Turns to meters
   print("Meters: " + str(2.3806*fturn))
   
   time.sleep(0.1)

