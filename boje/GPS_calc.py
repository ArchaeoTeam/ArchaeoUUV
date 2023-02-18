from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import io, time, sys, argparse, math, serial, socket, math, os
from datetime import datetime, timedelta
from subprocess import Popen
import pynmea2
import requests

import uvicorn
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse, FileResponse
from fastapi_versioning import VersionedFastAPI, version
from pydantic import BaseModel
import os
import threading
import datetime
import os
import time
from typing import Any, Dict, Optional


# Zeuch für den Webserver

class TextData(BaseModel):
    data: str

app = FastAPI(
    title="GPS_Calc",
    description="GPS_Calc",
)

@app.get("/load_calib_ph", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_ph() -> Any:
    data = "Test"

    return data
    
@app.post("/setTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setTime() -> Any:
    print("Zeit synchronisieren")
    return "ok"

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"



# Bis hier Webserver





alt_url = "http://192.168.2.2:4777/mavlink/VFR_HUD/"


def decTodms(deg):
     d = int(deg)
     md = abs(deg - d) * 60
     m = int(md)
     s = (md - m) * 60
     return "{:03d}{:07.4f}".format(d, m + (s / 60))

def add_offset(lat,lng, ang, os):

    #CALCULATION
    lat0 = math.cos(math.pi / 180.0 * lat)
    lng_new = lng + (180/math.pi) * (os / 6378137) / math.cos(lat0) * math.cos(ang)
    lat_new = lat  + (180/math.pi) * (os / 6378137) * math.sin(ang)

    return [lat_new,lng_new]

def parseArguments():
    if len(sys.argv)>1 and sys.argv[1].split("=")[0]=="-s":
        try:
            string_length = float(sys.argv[1].split("=")[1])
            return string_length
        except ValueError:
            print("No String length defined ( -s=2.0 )")
            return 2.0
    else:
        print("No String length defined ( -s=2.0 ) -2")
        return 2.0

def start_mavProxy():
    # mavproxy.py --out udp:127.0.0.1:14550 --show-errors --baudrate 115200
    print ("Start MavProxy Server...")
    #logfile = open("./mavprxy.log", "w")
    Popen(["mavproxy.py", "--out", "udp:127.0.0.1:14550"], shell=True)#,stdout=logfile)
    time.sleep(1)
    server_proc = Popen(["mavproxy.py", "--out", "udp:127.0.0.1:14550"], shell=True)#,stdout=logfile)
    time.sleep(30)
    return server_proc

def isclose(a,b,tolerance):
    return True if max(a,b) - min(a,b) < tolerance else False

###MAIN ENTRY POINT###
def main():
    string_length = parseArguments()
    print("String length:"+str(string_length))

    try:
        # Setup GPS serial port
        boje_serial_port = "/dev/ttyUSB0"
        ser_boje = serial.Serial(boje_serial_port, baudrate=115200,timeout=0.5)
    except Exception as e:
        print("No USB Device connected!")

    #   GC socket
    GC_IP = "192.168.2.1"
    GC_PORT = 27000
    sock_gc = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

    #   Open socket to Boot-pi
    BOOT_IP = "192.168.2.2"
    BOOT_PORT = 27000
    sock_boot = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

    # Connect to the MavProxies
    #start_mavProxy()
    print ("Connecting boje...")
    connection_boje = 'localhost:14550'
    boje = None
    try:
        boje = connect(connection_boje,"baud=57600")#, wait_ready=False)
    except Exception as e:
        print("Connecting to BOJE failed...Restarting...")
        os.system("sudo killall -SIGKILL mavproxy.py")
    #boje.wait_ready(True, timeout=180)
    

    while boje != None:
        time.sleep(0.1)
        #get BOOT parameter
        #try:
        #    resp = requests.get(alt_url + "alt")
        #    depth = float(resp.content)
        #    resp = requests.get(alt_url + "heading")
        #    angle_boot = float(resp.content)
        #    resp = requests.get(alt_url + "groundspeed")
        #    speed_boot = float(resp.content)
        #except Exception as e:
        #    print("No Data from BOOT!\n")

        #get BOJE parameter
        latitude = boje.location.global_frame.lat
        longitude = boje.location.global_frame.lon
        lat_dir = 'N' if latitude > 0 else 'S'
        lon_dir = 'E' if longitude > 0 else 'W'
        angle_boje = math.radians(boje.heading)
        speed_boje = boje.groundspeed
        newTime = datetime.now().strftime("%H%M%S.%f")
        
        if longitude == None or latitude == None:
            print("!NO GPS!")
        #offset = math.sqrt((string_length**2)-(depth**2))
        
        print("newLatlng: "+ str([latitude, longitude]))
        #Generate NMEA sentence and send it to UUV
        GPS_boje = pynmea2.GGA('GP', 'GGA', (newTime , decTodms(latitude), lat_dir,decTodms(longitude), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(float(boje.gps_0.eph)/100), '0', 'M', '0', 'M', '', ''))
        GSA_boje = pynmea2.GSA('GP', 'GSA', (boje.mode.name[1], str(boje.gps_0.fix_type),'17','15','19','24','32','10','12','25','','','','','0',str(boje.gps_0.eph ),str(10)))
        print("Sending: ")
        print(str(GPS_boje)+"\n")
        print(str(GSA_boje)+"\n")
        sock_boot.sendto(bytes(str(GPS_boje)+"\n",encoding='utf8'), (BOOT_IP, BOOT_PORT))
        sock_boot.sendto(bytes(str(GSA_boje)+"\n",encoding='utf8'), (BOOT_IP, BOOT_PORT))

        ##Correct if necessary
        #if (isclose(speed_boje, speed_boot, 0.2)):
        #    newLatLng = add_offset(latitude, longitude, angle_boot, offset)
        #    print("newLatlng: "+ str(newLatLng))
        #    #Generate NMEA sentence and send it to UUV
        #    GPS_boje = pynmea2.GGA('GP', 'GGA', (newTime , decTodms(newLatLng[0]), lat_dir,decTodms(newLatLng[1]), lon_dir, str(boje.gps_0.fix_type), str(boje.gps_0.satellites_visible), str(float(boje.gps_0.eph)/100), '0', 'M', '0', 'M', '', ''))
        #    GSA_boje = pynmea2.GSA('GP', 'GSA', (boje.mode.name[1], str(boje.gps_0.fix_type),'17','15','19','24','32','10','12','25','','','','','0',str(boje.gps_0.eph ),str(10)))
        #    print("Sending: ")
        #    print(str(GPS_boje)+"\n")
        #    print(str(GSA_boje)+"\n")
        #    sock_boot.sendto(bytes(str(GPS_boje)+"\n"), (BOOT_IP, BOOT_PORT))
        #    sock_boot.sendto(bytes(str(GSA_boje)+"\n"), (BOOT_IP, BOOT_PORT))
    print("Connection lost")


def loop():
    while True:
        try:
            main()
        except Exception as e:
            print("\nMain Loop Failed: \n" + str(e) + "\n")

thread_Loop = threading.Thread(target=loop, args=(), daemon=True)
thread_Loop.start()




uvicorn.run(app, host="0.0.0.0", port=80, log_config=None)

#        depth = boot.location.global_relative_frame.alt
#        compass_boot = boot.heading
#        speed_boot = boot.groundspeed
#        #GPS_boje_data = pynmea2.parse(ser.readline())
#        GPS_boje_data = pynmea2.parse()
#        depth = 1.5
#        offset = math.sqrt((string_length**2)-(depth**2))       
#        compass_boje = boje.heading
#        speed_boje = boje.groundspeed
#        angle = Math.toRadians(compass_boot)
#        print("Battery: %s" % boje.battery)
#        #print("offset: "+str(offset))
#        print("compass: "+str(compass_boje))
#        print("speed: "+str(speed_boje))
#        print("GPS: %s" % boje.gps_0)
#        print("Location: %s" % boje.location.global_frame)
#
#        if (math.isclose(speed_boje, speed_boot,rel_tol=0.2) and (compass_boot - compass_boje) < 5 ) : #roughly same speed in same direction
#                    GPS_boot_new = add_offset(GPS_boje_data, angle, offset)
#
#                    #send corrected data to uboot
#                    sock_boot.sendto(bytes(str(GPS_boot_new)), (BOOT_IP, UDP_PORT))
#                    #send og data
#                    sock_gc.sendto(bytes(str(GPS_boje_data)))
#
#        
#            #except KeyboardInterrupt:
#                #ctrl+c cleanup
#                
#                #server_proc.kill()
#
#
#            # Display basic boje state
#            #print "==========SAMPLE VALUES=========="
#            #print " Type: %s" %boje._vehicle_type
#            #print " Armed: %s" % boje.armed
#            #print " System status: %s" % boje.system_status.state
#            #print " GPS: %s" % boje.gps_0
#            #print " Alt: %s" % boje.location.global_relative_frame.alt
#            #print " Battery: %s" % boje.battery
#            #print " Groundspeed : %s" % boje.groundspeed
#            #print " heading : %s" % boje.heading
#            #print "================================="
#            #time.sleep(3)
#    except pynmea2.ParseError as e:
#        print('Parse error: {}'.format(e))
#        continue
#    #except serial.SerialException:
#    #    print("Couldnt send to the socket-server")
#    #    continue