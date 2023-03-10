import os
import spidev #import the SPI library for RB Pi 4 B board
import time #import the Timing library for RB Pi 4 B board
import datetime
import math
import threading

from urllib.parse import urlparse
import requests
import socket, serial
from htu21 import HTU21
import pynmea2
from csv_logger import CsvLogger

import uvicorn
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse, FileResponse
from fastapi_versioning import VersionedFastAPI, version
from pydantic import BaseModel

import numpy as np
from typing import Any, Dict, Optional

print("Starting GNSS Correction Service\nCalculating GDAL Geometry...")
from osgeo.ogr import Geometry, wkbPoint
from osgeo.osr import SpatialReference, SpatialReference, CoordinateTransformation

# Zeuch für den Webserver
dgps_nmea_obj=None
FIXLAT=51.0351205
FIXLON=13.7356563
RTKLAT=1
RTKLON=1
RTKX=1
RTKY=1

rover_nmea_obj=None

depth=1.5
compass=0
distance=0
correction=1

PrevDirection=0
PrevUTMX=-1
PrevUTMY=-1
UTMX=1
UTMY=1
BojeLat=1
BojeLon=1
correctedLat=1
correctedLon=1
Accuracy=1

htu = HTU21()



enable_correction=True
enable_RTK=False

class TextData(BaseModel):
    data: str

app = FastAPI(
    title="GPS_Calc",
    description="GPS_Calc",
)




# GPS Data


@app.get("/GPSLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return BojeLat

@app.get("/GPSLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return BojeLon

@app.get("/cGPSPosLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return correctedLat

@app.get("/cGPSPosLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return correctedLon

@app.get("/UTMX", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return UTMX

@app.get("/UTMY", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return UTMY





# correction Data


@app.get("/Depth", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return depth

@app.get("/Lenght", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return distance

@app.get("/Correction", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return correction

@app.get("/Direction", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return PrevDirection

@app.get("/Compass", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return compass

@app.get("/Accuracy", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return Accuracy





#DGPS Data


@app.get("/DGPSfixLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   #print("data")
   return FIXLAT

@app.get("/DGPSfixLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return FIXLON

@app.get("/DGPSposLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return RTKLAT

@app.get("/DGPSposLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return RTKLON

@app.get("/DGPScorrX", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return RTKX

@app.get("/DGPScorrY", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return RTKY


@app.get("/GetTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=datetime.datetime.now().strftime("%H:%M:%S  on  %d.%m.%Y")
   return str(x)


@app.get("/GetTemp", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(htu.read_temperature(),1)
   #print("T=",x)
   return str(x)

@app.get("/GetHum", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(htu.read_humidity(),1)
   #print("H=",x)
   return str(x)


@app.get("/GetDGPS", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_RTK
   return str(enable_RTK)

@app.get("/Getcorr", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_correction
   return str(enable_correction)

@app.post("/setDGPSLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(setDGPSLat: TextData) -> Any:
    global FIXLAT
    FIXLAT=setDGPSLat.data
    print("FIXLAT=",FIXLAT)
    return "ok"

@app.post("/setDGPSLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(setDGPSLon: TextData) -> Any:
    global FIXLON
    FIXLON=str(setDGPSLon.data)
    print("FIXLON=",FIXLON)
    return "ok"


@app.post("/enableDGPS", status_code=status.HTTP_200_OK)
@version(1, 0)
async def set(setDGPS: TextData) -> Any:
   global enable_RTK
   if int(setDGPS.data) == 0:
      enable_RTK = False
      print("DGPS aus")
   else:
      enable_RTK = True
      print("DGPS an")
   return "ok"




@app.post("/enableCalc", status_code=status.HTTP_200_OK)
@version(1, 0)
async def enableCalc(setDGPS: TextData) -> Any:
   global enable_correction
   if int(setDGPS.data) == 0:
      enable_correction=False
      print("corr aus")
   else:
      enable_correction=True
      print("corr an")
   return "ok"

@app.post("/restart", status_code=status.HTTP_200_OK)
@version(1, 0)
async def restart() -> Any:
    print("Script neustarten")
    os.system("sudo killall python3")
    return "ok"


    
@app.post("/setTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setTime() -> Any:
    print("Zeit synchronisieren")
    os.system("./timesync.sh")
    return "ok"

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"



# Bis hier Webserver


correction_possible = True

# DGPS
source = SpatialReference()
source.ImportFromEPSG(4326) # WGS84
target = SpatialReference()
target.ImportFromEPSG(5556) # UTM zone 32N

transform = CoordinateTransformation(source, target)
transformback = CoordinateTransformation(target, source)
point = Geometry(wkbPoint)
RTKpoint = Geometry(wkbPoint)
RTK = False



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
header = ['date', 
          'NMEA_boje', 
          'NMEA_corrected', 
          'NMEA_RTK', 
          'tether_length', 
          'compass_boot', 
          'move_direction', 
          'depth', 
          'accuracy', 
          'BojeLat','BojeLng','CoorectLat','CorrectLng','RTKLat','RTKLon']
csvlogger = CsvLogger(filename=log_filepath,
                      delimiter=delimiter,
                      max_files=50,
                      fmt='%(asctime)s.%(msecs)03d;%(message)s',
                      datefmt='%Y-%m-%d %H:%M:%S',
                      header=header)

def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s / 60))

def update_boot_values():
   fail_counter = 0

   while True:
      global depth
      global compass
      global correction_possible

      try:
         alt = float(requests.get(alt_url).text)
         #depth = 0.0 if alt < 0.0 else alt  #the ROV cannot fly yet
         compass = float(requests.get(compass_url).text)
         fail_counter = 0
      except requests.exceptions.RequestException as e:  # This is the correct syntax
         print("Failed to GET depth and compass values from ROV...")
         fail_counter += 1
         if fail_counter > 10:
            correction_possible = False
            print("No connection to ROV. GNSS correction is not possible!")
      time.sleep(0.1)

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

      time.sleep(0.01)

def readSerialNMEA(ser):
   global correction_possible

   while True:
      try: 
         line = ser.readline().decode('ascii', errors='replace')
         if (line != None and (line.startswith('$GNGGA') or line.startswith('$GPGGA'))):    
            return line
      except serial.SerialException as e:
         print("There is no new data from serial port")
         #correction_possible = False
         return None
      except TypeError as e:
         print("Disconnect of USB->UART occured")
         correction_possible = False
         return None

def send_RTK():
    while True:
        time.sleep(5)
        os.system("./sendRTK.sh")
        time.sleep(1)

def rec_Rover():
   global rover_nmea_obj

   UDP_IP = "192.168.2.3"
   UDP_PORT = 28001
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP
   sock.bind((UDP_IP, UDP_PORT))
   sock.listen(1)

   print("Rover-Server started")
   while True:
      #data, addr = sock.recvfrom(1024)
      connection, client = sock.accept()
      try:
         print("Connected to client IP: {}".format(client))
         
         # Receive and print data 32 bytes at a time, as long as the client is sending something
         while True:
            data = connection.recv(96)
            print("Received RTK: {}".format(data))

            if not data:
                  break
            #PARSE NMEA
            try:
               rover_nmea_obj = pynmea2.parse(data.decode('ascii'))
            except pynmea2.ParseError as e:
               print("Parse error: {0}".format(e))
               continue


      finally:
         connection.close()

def rec_RTK():
   global RTK
   global RTKLON
   global FIXLON
   global RTKLAT
   global FIXLAT
   global RTKX
   global RTKY

   UDP_IP = "192.168.2.3"
   UDP_PORT = 28002
   #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP
   sock.bind((UDP_IP, UDP_PORT))
   sock.listen(1)

   print("RTK-Server started")
   while True:
      #data, addr = sock.recvfrom(1024)
      connection, client = sock.accept()
      try:
         print("Connected to client IP: {}".format(client))
         
         # Receive and print data 32 bytes at a time, as long as the client is sending something
         while True:
            data = connection.recv(96)
            print("Received RTK: {}".format(data))

            if not data:
                  break
            #PARSE NMEA
            try:
               dgps_nmea_obj = pynmea2.parse(data.decode('ascii'))
            except pynmea2.ParseError as e:
               print("Parse error: {0}".format(e))
               continue
            
            # Base Station Koordinaten
            RTKLAT=dgps_nmea_obj.latitude
            RTKLON=dgps_nmea_obj.longitude

            #Fixpunkt Koordinaten TODO: Als Parameter einstellbar machen


            #Prüfen ob die Eingestellte RTK Koordinate richtig sein kann
            if abs(RTKLAT-FIXLAT) < 100:
               if abs(RTKLON-FIXLON) < 100:
                  RTK=True
                  RTKpoint(FIXLAT-RTKLAT,FIXLON-RTKLON)
                  RTKpoint.Transform(transform)
                  RTKX=RTKpoint.GetX()
                  RTKY=RTKpoint.GetY()

                  print("DGPS Lat/Lng: ", RTKLAT, RTKLON)
                  print("RTK X/Y: ", RTKX, RTKY)
                  
               else: 
                  RTK=False
                  print("RTK fixpunkt falsch")
            else: 
               RTK=False
               print("RTK fixpunkt falsch")

      finally:
         connection.close()
      # data = sock.recv(1024)
      
      

def getAccuracyEquip(distance, depth):
   #--> Bestimme Genauigkeit im schnitt 2cm pro meter 2cm bis 10m danach 2m --> TODO: Why these values?
   if depth < 20:
      return math.sqrt((distance * 0.02)*(distance * 0.02) + 0.15 * 0.15)
   else:
      return math.sqrt((distance * 0.02)*(distance * 0.02) + 2 * 2)

def sendNMEAtoROV(nmea):
   global BOOT_IP
   global BOOT_PORT

   print("Sending to ROV "+BOOT_IP+":"+str(BOOT_PORT) + "...")
   sock_boot.sendto(bytes(str(nmea)+"\n",encoding='utf8'), (BOOT_IP, BOOT_PORT))

#___________________________MAIN_______________________________


print("Starting Worker Threads...")

#start depth & compass thread
thread_boot = threading.Thread(target=update_boot_values, args=(), daemon=True)
thread_boot.start()

#start encoder thread
thread_encoder = threading.Thread(target=update_encoder_values, args=(), daemon=True)
thread_encoder.start()

#start DGPS thread

print("DGPS ACTIVE")
thread_encoder = threading.Thread(target=rec_RTK, args=(), daemon=True)
thread_encoder.start()

thread_encoder = threading.Thread(target=rec_Rover, args=(), daemon=True)
thread_encoder.start()


print("Waiting for GGA-Messages...")
counter = 0

def main():
   global UTMX
   global counter
   global UTMY
   global correction
   global distance
   global compass
   global depth
   global BojeLat
   global BojeLon
   global correctedLat
   global correctedLon
   global Accuracy
   global PrevUTMX
   global PrevUTMY
   global PrevDirection
   while True:
      #correction_possible = True
      ####GET GGA FROM SERIAL
      
      #TODO: Manchmal kommt hier ein None durch, wieso?
      nmea_str = readSerialNMEA(ser)
      if correction_possible:
         print(str(counter)+"\n----------------------------------------")
         print(nmea_str)

         try:
            nmea_obj = pynmea2.parse(nmea_str)
         except pynmea2.ParseError as e:
            print("Parse error: {0}".format(e))
            continue
            
         ####GET GPS FROM RTK 

      

         ####CORRECT GPS WITH RTK
         #RTKX= 0	#--> RTK Offset X (DUMMY)
         #RTKY= 0	#--> RTK Offset Y (DUMMY)

         distance = 2
         ####Calculate Offset with Pythagoras | distance² = depth² + offset²
         if depth > distance:
            print("skipping bc depth bigger distance...")
            print("distance=" + str(distance) + "\ndepth=" + str(depth))
            csvlogger.info([nmea_str.rstrip(), "dist-depth-error", 0, distance, compass, depth, Accuracy])
            sendNMEAtoROV(nmea_obj)
            continue

         correction = math.sqrt(math.pow(distance,2) - math.pow(depth,2))
         print("Correction-offset:" + str(correction) + " m")




         ####CONVERT TO UTM
         try:
            BojeLat=nmea_obj.latitude
            BojeLon=nmea_obj.longitude
            
            #--> Coordinates to gdal point
            point.AddPoint(nmea_obj.latitude, nmea_obj.longitude)

            # copmute a direction from the last point to the current point
            if PrevUTMX == -1 or PrevUTMY == -1:
               PrevUTMX = point.GetX()
               PrevUTMY = point.GetY()
               print("SKIP CORRECTION DUE TO FIRST POINT")
               continue

            delta_x = point.GetX() - PrevUTMX
            delta_y = point.GetY() - PrevUTMY
            direction = (np.arctan2(delta_x, delta_y) * 180 / np.pi + 360) % 360
            print("direction: " + str(direction))

            #stash the current point for the next iteration
            PrevUTMX = point.GetX()
            PrevUTMY = point.GetY()
            PrevDirection = direction

            #offset meters to UTM
            UTMY=math.sin(direction)*correction
            UTMX=math.cos(direction)*correction

            if enable_RTK:
               point.AddPoint(point.GetX()+RTKX, point.GetY()+RTKY)
               Accuracy = getAccuracyEquip(distance, depth) + 0,35
               print("berechne DGPS")
            else:
               Accuracy = getAccuracyEquip(distance, depth) + 3
            print("Accuracy:" + str(Accuracy) + " m")

            #--> Transform to UTM
            point.Transform(transform)

            #--> Actual Correction
            if enable_correction:            
               point.AddPoint(point.GetX()+UTMX, point.GetY()+UTMY)
               print("berechne Korrektur")
            else: print("SKIP CORRECTION")

            correctedLat=point.GetX()
            correctedLon=point.GetY()

            if enable_RTK:
               point.AddPoint(point.GetX()+RTKX, point.GetY()+RTKY)
            
            else: print("SKIP DGPS")
            

            #--> Transform back to WGS84
            point.Transform(transformback)
         except:
            print("GDAL ERROR...skip")
            continue 
         if enable_correction:

            try:
               ####GENERATE GGA
               new_nmea = pynmea2.GGA('GN', 'GGA', (nmea_obj.timestamp.strftime("%H%M%S.000"), 
                                                            decTodms(point.GetX()), 
                                                            str(nmea_obj.lat_dir), 
                                                            decTodms(point.GetY()),
                                                            str(nmea_obj.lon_dir), 
                                                            str(2),                         # Fix Type 2 = D-GPS
                                                            str(nmea_obj.num_sats), 
                                                            str(nmea_obj.horizontal_dil), 
                                                            str(0), 
                                                            str(nmea_obj.altitude_units), 
                                                            str(nmea_obj.geo_sep), 
                                                            str(nmea_obj.geo_sep_units), 
                                                            str(nmea_obj.age_gps_data),     # Age of correction data?
                                                            str(nmea_obj.ref_station_id)))
            except:
               print("Error while generating GGA")
               continue
            print("\nNew GGA:\n"+str(new_nmea))
            sendNMEAtoROV(new_nmea)
         #    ['date', 
         #  'NMEA_rover', 
         #  'NMEA_corrected', 
         #  'NMEA_base', 
         #  'tether_length', 
         #  'compass_boot', 
         #  'move_direction', 
         #  'depth', 
         #  'accuracy', 
         #  'BojeLat','BojeLng','CoorectLat','CorrectLng','RTKLat','RTKLon']
            csvlogger.info([
               nmea_str.rstrip(), 
               str(new_nmea), 
               str(rover_nmea_obj), 
               str(distance), 
               str(compass), 
               str(direction), 
               str(depth), 
               str(Accuracy),
               str(BojeLat), str(BojeLon), str(correctedLat), str(correctedLon), str(rover_nmea_obj.latitude), str(rover_nmea_obj.longitude)])
         else:
            sendNMEAtoROV(nmea_str)
         ####LOG EVERYTHING TO CSV
         
      
         ####SEND TO ROV
         
         counter+=1
         print("----------------------------------------")
      else:
         print("#Correction skipped... something missing here...#")
         sendNMEAtoROV(nmea_str)

#______________________MAIN LOOP________________________
def loop():
    while True:
        try:
            main()
        except Exception as e:
            print("\nMain Loop Failed: \n" + str(e) + "\n")

thread_Loop = threading.Thread(target=loop, args=(), daemon=True)
thread_Loop.start()



# Start Webserver
uvicorn.run(app, host="0.0.0.0", port=81, log_config=None)
