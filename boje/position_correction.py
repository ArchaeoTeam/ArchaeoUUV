import os, math
import numpy as np
import socket, serial, spidev
import time
import datetime
import threading

from urllib.parse import urlparse
import requests
from htu21 import HTU21
import pynmea2
from csv_logger import CsvLogger
from typing import Any, Dict, Optional

print("Starting GNSS Correction Service\nCalculating GDAL Geometry...")
from osgeo.ogr import Geometry, wkbPoint
from osgeo.osr import SpatialReference, SpatialReference, CoordinateTransformation

source = SpatialReference()
source.ImportFromEPSG(4326)  # WGS84
target = SpatialReference()
target.ImportFromEPSG(5556)  # UTM zone 32N

transform = CoordinateTransformation(source, target)
transformback = CoordinateTransformation(target, source)
point = Geometry(wkbPoint)
DGPSpoint = Geometry(wkbPoint)


class Location:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon


htu = HTU21()
# ENCODER CONSTANTS
AMT22_NOP = 0  # command to read the position of the encoder
AMT22_NOP = 0x00
AMT22_RESET = 0x60
AMT22_ZERO = 0x70
AMT22_READ_TURNS = 0xA0
NEWLINE = 0x0A
TAB = 0x09
spi = spidev.SpiDev()  # create the spi object
spi.open(0, 0)  # SPI port 0, CS 0
speed_hz = 500000  # setting the speed in hz

ALTITUDE_URL = "http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/AHRS2/message/altitude"
COMPASS_URL = "http://192.168.2.2:6040/mavlink/vehicles/1/components/1/messages/VFR_HUD/message/heading"

# ENCODER VALUES
delay_us = 3  # setting the delay in microseconds
turns = 0  # number of turns of the wheel
rotation = 0  # position of wheel
distance = 0  # in meters between buoy and ROV
result = []

fix_point = Location(51.0351205, 13.7356563)
rtk_rover = Location(0, 0)
rtk_correction_utm = Location(0, 0)  # y = lat, x = lon!!!
boje_position = Location(0, 0)
corrected_position = Location(0, 0)
prev_boje_position = Location(0, 0)
correction_offset_utm = Location(0, 0)  # y = lat, x = lon!!!

rover_nmea = None
dgps_nmea = None

depth = 1.5
rov_compass = 0
distance = 0  # distance between buoy and ROV
correction_offset = 0
accuracy = 0
direction = 0
prev_direction = -1

enable_correction = True
enable_RTK = False

correction_possible = True
rtk_possible = False

# SERIAL GPS
ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=5)

# ROV SOCKET
BOOT_IP = "192.168.2.2"
BOOT_PORT = 27000
sock_boot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP

# LOGGING
log_filepath = "logs/GNSS_" + time.strftime("%Y%m%d-%H%M%S") + ".csv"
delimiter = ";"
header = [
    "date",
    "NMEA_boje",
    "NMEA_corrected",
    "NMEA_RTK",
    "tether_length",
    "compass_boot",
    "move_direction",
    "depth",
    "accuracy",
    "DGPS+possible",
    "Correction+possible" "BojeLat",
    "BojeLng",
    "CoorectLat",
    "CorrectLng",
    "RTKLat",
    "RTKLon",
]
csvlogger = CsvLogger(
    filename=log_filepath,
    delimiter=delimiter,
    max_files=50,
    fmt="%(asctime)s.%(msecs)03d;%(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    header=header,
)


def decTodms(deg):
    d = int(deg)
    md = abs(deg - d) * 60
    m = int(md)
    s = (md - m) * 60
    return "{:03d}{:07.4f}".format(d, m + (s / 60))


def getAccuracyEquip(distance, depth):
    # --> Bestimme Genauigkeit im schnitt 2cm pro meter 2cm bis 10m danach 2m --> TODO: Why these values?
    if depth < 20:
        return math.sqrt((distance * 0.02) * (distance * 0.02) + 0.15 * 0.15)
    else:
        return math.sqrt((distance * 0.02) * (distance * 0.02) + 2 * 2)


def sendNMEAtoROV(nmea):
    global BOOT_IP
    global BOOT_PORT

    print("Sending to ROV " + BOOT_IP + ":" + str(BOOT_PORT) + "...")
    sock_boot.sendto(bytes(str(nmea) + "\n", encoding="utf8"), (BOOT_IP, BOOT_PORT))
    print("Done!")


# gives a direction in degree from 2 lat/lon Locations
def getDirection(point_curr, point_prev):
    # --> Radiant
    direction = math.atan2(
        point_curr.lon - point_prev.lon, point_curr.lat - point_prev.lat
    )
    # --> Degree
    direction = math.degrees(direction)
    # --> 0-360
    direction = round(direction, 2)
    if direction < 0:
        direction += 360
    return direction


def updateRovValues():
    fail_counter = 0

    while True:
        global depth
        global rov_compass
        global correction_possible

        try:
            alt = float(requests.get(ALTITUDE_URL).text)
            # depth = 0.0 if alt < 0.0 else alt  #the ROV cannot fly yet
            rov_compass = float(requests.get(COMPASS_URL).text)
            fail_counter = 0
        except requests.exceptions.RequestException as e:  # This is the correct syntax
            print("Failed to GET depth and compass values from ROV...")
            fail_counter += 1
            if fail_counter > 10:
                correction_possible = False
                print("No connection to ROV. GNSS correction is not possible!")
        time.sleep(0.1)


def updateEncoderValues():
    while True:
        global turns
        global rotation
        global result
        global distance

        result = spi.xfer2(
            [AMT22_NOP, AMT22_READ_TURNS, AMT22_NOP, AMT22_NOP], speed_hz, delay_us
        )
        rotation = (
            16383 - ((result[0] & 0b111111) << 8) + result[1]
        )  # 0 - 16383 Pro umdrehung
        turns = 255 - result[3]
        # Convert RAW encoder data to Wheel-Turns float
        fturn = turns + ((rotation - 4500) / 16383)
        # Convert Turns to Distance(Buoy, UUV)
        distance = fturn / 2.3806
        time.sleep(0.01)


def readSerialNMEA(ser):
    global correction_possible

    while True:
        try:
            line = ser.readline().decode("ascii", errors="replace")
            if line != None and (
                line.startswith("$GNGGA") or line.startswith("$GPGGA")
            ):
                return line
        except serial.SerialException as e:
            print("There is no new data from serial port")
            # correction_possible = False
            return None
        except TypeError as e:
            print("Disconnect of USB->UART occured")
            correction_possible = False
            return None

def recRover():
    global rover_nmea

    UDP_IP = "192.168.2.3"
    UDP_PORT = 28001
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
    sock.bind((UDP_IP, UDP_PORT))
    sock.listen(1)

    print("Rover-Server started")
    while True:
        connection, client = sock.accept()
        try:
            print("Connected to client IP: {}".format(client))

            # Receive and print data 96 bytes at a time, as long as the client is sending something
            while True:
                data = connection.recv(96)
                print("Received RTK: {}".format(data))

                if not data:
                    break
                # PARSE NMEA
                try:
                    rover_nmea = pynmea2.parse(data.decode("ascii"))
                except pynmea2.ParseError as e:
                    print("Parse error: {0}".format(e))
                    continue
        finally:
            connection.close()


def recRTK():
    global rtk_possible
    global fix_point
    global dgps_nmea

    UDP_IP = "192.168.2.3"
    UDP_PORT = 28002
    # sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
    sock.bind((UDP_IP, UDP_PORT))
    sock.listen(1)

    print("RTK-Server started")
    while True:
        # data, addr = sock.recvfrom(1024)
        connection, client = sock.accept()
        try:
            print("Connected to client IP: {}".format(client))

            # Receive and print data 32 bytes at a time, as long as the client is sending something
            while True:
                data = connection.recv(96)
                print("Received RTK: {}".format(data))

                if not data:
                    break
                # PARSE NMEA
                try:
                    dgps_nmea = pynmea2.parse(data.decode("ascii"))
                except pynmea2.ParseError as e:
                    print("Parse error: {0}".format(e))
                    continue

                # Prüfen ob die Eingestellte RTK Koordinate richtig sein kann
                if abs(dgps_nmea.latitude - fix_point.lat) < 100:
                    if abs(dgps_nmea.longitude - fix_point.lon) < 100:
                        rtk_possible = True
                        DGPSpoint(
                            fix_point.lat - dgps_nmea.latitude,
                            fix_point.lon - dgps_nmea.longitude,
                        )
                        DGPSpoint.Transform(transform)

                        print("DGPS Lat/Lng: ", dgps_nmea.latitude, dgps_nmea.longitude)
                        print("RTK X/Y: ", DGPSpoint.GetX(), DGPSpoint.GetY())

                    else:
                        rtk_possible = False
                        print("RTK fixpunkt falsch")
                else:
                    rtk_possible = False
                    print("RTK fixpunkt falsch")

        finally:
            connection.close()


#####################################################################################
# ______________________________________MAIN__________________________________________
#####################################################################################
print("Starting Worker Threads...")
# ROV THREAD (DEPTH, COMPASS)
thread_boot = threading.Thread(target=updateRovValues, args=(), daemon=True)
thread_boot.start()

# ENCODER THREAD
thread_encoder = threading.Thread(target=updateEncoderValues, args=(), daemon=True)
thread_encoder.start()

thread_encoder = threading.Thread(target=recRTK, args=(), daemon=True)
thread_encoder.start()

thread_encoder = threading.Thread(target=recRover, args=(), daemon=True)
thread_encoder.start()


print("Waiting for GGA-Messages...")
counter = 0


def main():
    global counter
    global correction_offset_utm
    global correction_offset
    global prev_boje_position
    global boje_position
    global corrected_position
    global distance
    global rov_compass
    global depth
    global accuracy
    global direction
    global prev_direction
    global correction_possible

    print("Started Main Thread...")
    while True:
        # TODO: Manchmal kommt hier ein None durch, wieso?
        nmea_str = readSerialNMEA(ser)
        if correction_possible:
            print(str(counter) + "\n----------------------------------------")
            print(nmea_str)
            try:
                nmea_obj = pynmea2.parse(nmea_str)
                if type(nmea_obj) == type(None):
                    print("nmea_obj NoneType")
                    continue
            except pynmea2.ParseError as e:
                print("Parse error: {0}".format(e))
                continue

            ####Calculate Offset with Pythagoras | distance² = depth² + offset²
            if depth > distance:
                print("skipping bc depth bigger distance...")
                print("distance=" + str(distance) + "\ndepth=" + str(depth))
                csvlogger.info(
                    [
                        nmea_str.rstrip(),
                        "dist-depth-error",
                        0,
                        distance,
                        rov_compass,
                        depth,
                        accuracy,
                    ]
                )
                sendNMEAtoROV(nmea_obj)
                continue

            correction_offset = math.sqrt(math.pow(distance, 2) - math.pow(depth, 2))
            print("Correction-offset:" + str(correction_offset) + " m")

            ####CONVERT TO UTM
            try:
                boje_position.lat = nmea_obj.latitude
                boje_position.lon = nmea_obj.longitude
                print("Boje Lat/Lng: ", boje_position.lat, boje_position.lon)
                # --> Coordinates to gdal point
                point.AddPoint(nmea_obj.longitude, nmea_obj.latitude)

                # compute a direction from the last point to the current point
                if prev_boje_position.lat == -1 or prev_boje_position.lon == -1:
                    prev_boje_position.lon = point.GetX()
                    prev_boje_position.lat = point.GetY()
                    print("SKIP CORRECTION DUE TO FIRST POINT")
                    continue

                direction = getDirection(boje_position, prev_boje_position)
                print("direction: " + str(direction))

                # stash the current point for the next iteration
                prev_boje_position.lon = point.GetX()
                prev_boje_position.lat = point.GetY()

                prev_direction = direction

                ##################################### UTM TIME #####################################
                # --> Transform to UTM
                point.Transform(transform)

                # offset meters to UTM
                correction_offset_utm.lon = (
                    math.cos(direction) * correction_offset
                )  # --> UTM Offset X
                correction_offset_utm.lat = (
                    math.sin(direction) * correction_offset
                )  # --> UTM Offset Y

                if enable_RTK:
                    point.AddPoint(
                        point.GetX() + rtk_correction_utm.lon,
                        point.GetY() + rtk_correction_utm.lat,
                    )
                    accuracy = getAccuracyEquip(distance, depth) + 0, 35
                    print("berechne DGPS")
                else:
                    print("SKIP DGPS")
                    accuracy = getAccuracyEquip(distance, depth) + 3
                print("Accuracy:" + str(accuracy) + " m")

                # --> Actual Correction
                if enable_correction:
                    point.AddPoint(
                        point.GetX() + correction_offset_utm.lon,
                        point.GetY() + correction_offset_utm.lat,
                    )
                    print("berechne Korrektur")
                else:
                    print("SKIP CORRECTION")

                # --> Transform back to WGS84
                point.Transform(transformback)
                ################################## BACK TO WGS84 ###################################
                corrected_position = Location(point.GetY(), point.GetX())
            except:
                print("GDAL ERROR...skip")
                continue
            if enable_correction:
                try:
                    ####GENERATE GGA
                    new_nmea = pynmea2.GGA(
                        "GN",
                        "GGA",
                        (
                            nmea_obj.timestamp.strftime("%H%M%S.000"),
                            decTodms(point.GetY()),
                            str(nmea_obj.lat_dir),
                            decTodms(point.GetX()),
                            str(nmea_obj.lon_dir),
                            str(2) if enable_RTK and rtk_possible else str(6),  # Fix Type 2 = D-GPS; 6 = Estimated
                            str(nmea_obj.num_sats),
                            str(nmea_obj.horizontal_dil),
                            str(0),
                            str(nmea_obj.altitude_units),
                            str(nmea_obj.geo_sep),
                            str(nmea_obj.geo_sep_units),
                            str(nmea_obj.age_gps_data),  # Age of correction data?
                            str(nmea_obj.ref_station_id),
                        ),
                    )
                except Exception as e:
                    print("Error while generating GGA + " + str(e))
                    continue
                print("\nNew GGA:\n" + str(new_nmea))
                sendNMEAtoROV(new_nmea)

                csvlogger.info(
                    [
                        nmea_str.rstrip(),  #  'NMEA_boje',
                        str(new_nmea),  #  'NMEA_corrected',
                        str(rover_nmea),  #  'NMEA_RTK',
                        str(distance),  #  'tether_length',
                        str(rov_compass),  #  'compass_boot',
                        str(direction),  #  'move_direction',
                        str(depth),  #  'depth',
                        str(accuracy),  #  'accuracy',
                        str(enable_RTK) + "+" + str(rtk_possible),  #  'DGPS+possible'
                        str(enable_correction)
                        + "+"
                        + str(correction_possible),  # ,'Correction+possible'
                        #  'BojeLat','BojeLng','CoorectLat','CorrectLng','RTKLat','RTKLon']
                        str(boje_position.lat),
                        str(boje_position.lon),
                        str(corrected_position.lat),
                        str(corrected_position.lon),
                        "None", #TODO: Add this without error
                        "None",
                    ]
                )
            else:
                sendNMEAtoROV(nmea_str)
            counter += 1
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
