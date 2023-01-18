import spidev #import the SPI library for RB Pi 4 B board
import time #import the Timing library for RB Pi 4 B board
from urllib.parse import urlparse
import requests
import threading
import os, sys
import socket, serial
import math
import pynmea2
from csv_logger import CsvLogger
import logging

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


#ENCODER CONSTANTS
AMT22_NOP = 0 #command to read the position of the encoder
AMT22_NOP = 0x00
AMT22_RESET = 0x60
AMT22_ZERO = 0x70
AMT22_READ_TURNS = 0xA0
NEWLINE = 0x0A
TAB = 0x09
spi = spidev.SpiDev() #create the spi object
spi.open(0, 0) #SPI port 0, CS 0
speed_hz=500000 #setting the speed in hz

#ENCODER VALUES
delay_us=3 #setting the delay in microseconds
Accuracy = 0
turns = 0   #number of turns of the wheel
rotation = 0 #position of wheel
distance = 0 #in meters between buoy and ROV
result = []

alt_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/AHRS2/message/altitude"
compass_url="http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/VFR_HUD/message/heading"
depth = 0
compass = 0

#SERIAL GPS
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout = 5)

#ROV SOCKET
BOOT_IP = "192.168.2.2"
BOOT_PORT = 27000
sock_boot = socket.socket(socket.AF_INET, # Internet
                  socket.SOCK_DGRAM) # UDP

#LOGGING
log_filepath = "logs/GNSS_"+time.strftime("%Y%m%d-%H%M%S")+ ".csv"
delimiter = ';'
header = ['date', 'NMEA_rover', 'NMEA_corrected', 'NMEA_base', 'tether_length', 'compass', 'depth', 'accuracy']
csvlogger = CsvLogger(filename=log_filepath,
                      delimiter=delimiter,
                      max_files=50,
                      header=header)

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s / 60))

def update_boot_values():
   while True:
      global depth
      global compass

      alt = float(requests.get(alt_url).text)

      depth = 0.0 if alt < 0.0 else alt  #the ROV cannot fly yet
      compass = float(requests.get(compass_url).text)

def update_encoder_values():
   while True:
      global turns
      global rotation
      global result
      global distance

      result=spi.xfer2([AMT22_NOP, AMT22_READ_TURNS, AMT22_NOP, AMT22_NOP],speed_hz,delay_us)

      rotation=16383-((result[0] & 0b111111) << 8) + result[1] # 0 - 16383 Pro umdrehung

      turns=255-result[3]

      #Convert RAW encoder data to Wheel-Turns float
      fturn = turns+( (rotation-4500)/16383 )

      #Convert Turns to Distance(Buoy, UUV)
      distance = fturn/2.3806

      #lenght=(((calibturns-turns)*16383)-rotation+calibtotat)/370
      #print(lenght)

def readSerialNMEA(ser):
   try:
      while True:
         line = ser.readline().decode('ascii', errors='replace')
         if (line != None and line.startswith('$GNGGA')):    
            return line
   except:
      print("Got nothing")

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
        data, addr = sock.recvfrom(1024)
        print("received message: %s" % data)

        #TODO: What is this?
        RTKLAT=1
        RTKLON=2
        FIXLAT=51.07205984
        FIXLON=13.59835664

        #Prüfen ob die Eingestellte RTK Koordinate richtig sein kann
        if abs(RTKLAT-FIXLAT) < 100:
            if abs(RTKLON-FIXLON) < 100:
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

def getAccuracyEquip(distance, depth):
   #--> Bestimme Genauigkeit im schnitt 2cm pro meter 2cm bis 10m danach 2m --> TODO: Why these values?
   if depth < 20:
      return math.sqrt((distance * 0.02)*(distance * 0.02) + 0.15 * 0.15)
   else:
      return math.sqrt((distance * 0.02)*(distance * 0.02) + 2 * 2)


#___________________________MAIN_______________________________        
print("Starting Worker Threads...")

#start depth & compass thread
thread_boot = threading.Thread(target=update_boot_values, args=(), daemon=True)
thread_boot.start()

#start encoder thread
thread_encoder = threading.Thread(target=update_encoder_values, args=(), daemon=True)
thread_encoder.start()

print("Waiting for GGA-Messages...")

while True:
#GET GGA FROM SERIAL
   try:
      #TODO: Manchmal kommt hier ein None durch, wieso?
      nmea_str = readSerialNMEA(ser)
      print(nmea_str)
      nmea_obj = pynmea2.parse(nmea_str, check=False)

#GET GPS FROM RTK 
   #TODO: RTK, DGPS, NTRIP, RTCM message...? auf dem RTK läuft ein NTRIP-Server
   

#CORRECT GPS WITH RTK
      RTKX= 0	#--> RTK Offset X (DUMMY)
      RTKY= 0	#--> RTK Offset Y (DUMMY)


#Calculate Offset with Pythagoras | distance² = depth² + offset²
      correction = math.sqrt(math.pow(distance,2) - math.pow(depth,2))
      print("Correction-offset:" + str(correction) + "m")

#CONVERT TO UTM
      #offset meters to UTM
      UTMY=math.sin(compass)*correction
      UTMX=math.cos(compass)*correction

      #--> Coordinates to gdal point
      point.AddPoint(nmea_obj.latitude, nmea_obj.longitude)

      #--> Transform to UTM
      point.Transform(transform)

      if RTK:
         point.AddPoint(point.GetX()+RTKX, point.GetY()+RTKY)
         Accuracy = getAccuracyEquip(distance, depth) + 0,35
      else:
         Accuracy = getAccuracyEquip(distance, depth) + 3
      print("Accuracy:" + str(Accuracy) + "m")

      #--> Actual Correction
      point.AddPoint(point.GetX()+UTMX, point.GetY()+UTMY)

      #--> Transform back to WGS84
      point.Transform(transformback)
      
#GENERATE GGA
      new_nmea = pynmea2.GGA('GN', 'GGA', (nmea_obj.timestamp.strftime("%H%M%S.%f"), 
                                                   decTodms(point.GetX()), 
                                                   str(nmea_obj.lat_dir), 
                                                   decTodms(point.GetY()),
                                                   str(nmea_obj.lon_dir), 
                                                   str(2),                         # Fix Type 2 = D-GPS
                                                   str(nmea_obj.num_sats), 
                                                   str(nmea_obj.horizontal_dil), 
                                                   str(nmea_obj.altitude), 
                                                   str(nmea_obj.altitude_units), 
                                                   str(nmea_obj.geo_sep), 
                                                   str(nmea_obj.geo_sep_units), 
                                                   str(nmea_obj.age_gps_data),     # Age of correction data?
                                                   str(nmea_obj.ref_station_id)))
      print("\nNew GGA:\n"+str(new_nmea))

#LOG EVERYTHING TO CSV
      csvlogger.info([nmea_str, str(new_nmea), 0, distance, compass, depth, Accuracy])
   
#SEND TO ROV
      print("Sending to ROV "+BOOT_IP+":"+str(BOOT_PORT) + "...")
      sock_boot.sendto(bytes(str(new_nmea)+"\n",encoding='utf8'), (BOOT_IP, BOOT_PORT))

      print("----------------------------------------")

   except pynmea2.ParseError as e:
        print("Parse error: {0}".format(e))



#         os.system("clear")
#      #Print boot data
#      print("Tiefe")
#      print(depth)
#      print("Kompass")
#      print(compass)
#
#      #Print RAW encoder data
#      print("Result: " + str(result))
#      print("RAWRotation: " + str(rotation))
#      print("RAWTurns: " + str(turns))
#
#      #Convert RAW encoder data to Wheel-Turns float
#      fturn = turns+( (rotation-4500)/16383 )
#      print("Turns: " + str(fturn))
#
#      #Convert Turns to Distance(Buoy, UUV)
#      distance = fturn/2.3806
#      print("Meters: " + str(distance))
#
#      #Calculate Offset with Pythagoras | distance² = depth² + offset²
#      correction = math.sqrt(math.pow(distance,2) - math.pow(depth,2))
#      print("Correction-offset:" + str(correction))
#      
#      #Calc with compas in UTM
#      UTMY=math.sin(compass)*correction
#      UTMX=math.cos(compass)*correction
#      
#      #Angle 0 = cosinus 1 -- x
#      #Angle 90 = sinus 1 --> y
#      print("UTMX:" + str(UTMX))
#      print("UTMY:" + str(UTMY))
#      
#      # 
#      Lat="51.035540"
#      Lon="13.735870"
#      
#      # --> Bringe die Koordinaten in gdal
#      point.AddPoint(Lat, Lon)
#      #--> Transformiere in UTM
#      point.Transform(transform)
#
#      Accuracy = getAccuracy(distance, depth)
#
#      if RTK:
#         point.AddPoint(point.GetX()+RTKX, point.GetY()+RTKY)
#         Accuracy = Accuracy + 0,35
#      else:
#         Accuracy = Accuracy + 3
#         
#      point.AddPoint(point.GetX()+UTMX, point.GetY()+UTMY)
#      point.Transform(transformback)
#      
#      
#      Lat=point.GetX()
#      Lon=point.GetY()
#      Accuracy=Accuracy
#      #Hier umwandeln in NMEA und ans Boot senden bitte David
#      #Bitte im Paket angeben statt fix DGPS
#   
#      time.sleep(0.5)