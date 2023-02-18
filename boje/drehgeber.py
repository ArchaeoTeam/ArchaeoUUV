import spidev #import the SPI library for RB Pi 4 B board
import time #import the Timing library for RB Pi 4 B board
from urllib.parse import urlparse
import requests
import threading
import subprocess
import os
import socket
import math
import sys
from osgeo.ogr import Geometry, wkbPoint
from osgeo.osr import SpatialReference, SpatialReference, CoordinateTransformation

source = SpatialReference()
source.ImportFromEPSG(4326)
target = SpatialReference()
target.ImportFromEPSG(5556)

transform = CoordinateTransformation(source, target)
transformback = CoordinateTransformation(target, source)
point = Geometry(wkbPoint)
RTKpoint = Geometry(wkbPoint)
RTK = False
RTKLAT=0
RTKLON=0



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
Accuracy = 0
turns = 0
rotation = 0
result = []

alt_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/AHRS2/message/altitude"
compass_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/VFR_HUD/message/heading"
depth = 0
compass = 0

def update_boot_values():
   while True:
      global depth
      global compass
      depth2 = float(requests.get(alt_url).text)
      compass = float(requests.get(compass_url).text)
      if depth2 < 0.0:
         depth=0.0
      else:
         depth=depth2

def update_encoder_values():
   while True:
      global turns
      global rotation
      global result

      result=spi.xfer2([AMT22_NOP, AMT22_READ_TURNS, AMT22_NOP, AMT22_NOP],speed_hz,delay_us)

      rotation=16383-((result[0] & 0b111111) << 8) + result[1] # 0 - 16383 Pro umdrehung

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
        RTKLAT=1
        RTKLON=2
        FIXLAT=51,07205984
        FIXLON=13,59835664
        #Prüfen ob die Eingestellte RTK Koordinate richtig sein kann
        if abs(RTKLAT-FIXLAT) < 100
            if abs(RTKLON-FIXLON) < 100
               RTK=True
               RTKpoint(FIXLAT-RTKLAT,FIXLON-RTKLON)
               RTKpoint.Transform(transform)
               RTKX=RTKpoint.GetX()
               RTKY=RTKpoint.GetY()
               
            
            else: 
               RTK=False
               print("RTK fixpunkt falsch")
        else: 
           RTK=False
           print("RTK fixpunkt falsch")
            
        

    
        
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

   #Print encoder data
   print("Result: " + str(result))
   print("RAWRotation: " + str(rotation))
   print("RAWTurns: " + str(turns))

   #Print Turns
   fturn=turns+( (rotation-4500)/16383 )
   print("Turns: " + str(fturn))

   #Convert Turns to meters
   print("Meters: " + str(fturn/2.3806))
   #Abstand=sqrt(Länge Kabel^2-tiefe^2)
   #calc Correctionlenght
   correction=math.sqrt(math.pow(fturn/2.3806,2) - math.pow(depth,2))
   print("Correction:" + str(correction))
   
   #Calc with compas in UTM
   UTMY=math.sin(compass)*correction
   UTMX=math.cos(compass)*correction
   
   #winkel 0 = cosinus 1 -- x
   #winkel 90 = sinus 1 --> y
   print("UTMX:" + str(UTMX))
   print("UTMY:" + str(UTMY))
   
   # Hier erstmal beispieldaten
   Lat="51.035540"
   Lon="13.735870"
   
   # --> Bringe die Koordinaten in gdal
   point.AddPoint(Lat, Lon)
   #--> Transformiere in UTM
   point.Transform(transform)
   #--> Bestimme Genauigkeit im schnitt 2cm pro meter 2cm bis 10m danach 2m --> 
   Accuracy = math.sqrt((str(fturn/2.3806) * 0,02)*(str(fturn/2.3806) * 0,02) + 4)
   if depth < 20
      Accuracy = math.sqrt((str(fturn/2.3806) * 0,02)*(str(fturn/2.3806) * 0,02) + 0,15 *0,15)
   else:
      Accuracy = math.sqrt((str(fturn/2.3806) * 0,02)*(str(fturn/2.3806) * 0,02) + 2 * 2)

   
   if RTK = True:
      point.AddPoint(point.GetX()+RTKX, point.GetY()+RTKY)
      Accuracy = Accuracy + 0,35
   else:
      Accuracy = Accuracy + 3
      
   point.AddPoint(point.GetX()+UTMX, point.GetY()+UTMY)
   point.Transform(transformback)
   
   
   Lat=point.GetX()
   Lon=point.GetY()
   Accuracy=Accuracy
   #Hier umwandeln in NMEA und ans Boot senden bitte David
   #Bitte im Paket angeben statt fix DGPS
  
   time.sleep(0.5)

