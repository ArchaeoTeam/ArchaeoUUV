#! /usr/bin/env python3

from pathlib import Path
import statistics
import appdirs
import uvicorn
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse, FileResponse
from fastapi_versioning import VersionedFastAPI, version
from loguru import logger
from gpiozero import Servo
from unsync import unsync
from multiprocessing import Process
from smbus2 import SMBus

from pydantic import BaseModel
import os
import requests
import threading
import datetime
import asyncio
import json
import os
import time
from typing import Any, Dict, Optional
import sys
import board
import aiohttp
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO


user_config_dir = Path(appdirs.user_config_dir())
#os.mkdir(user_config_dir)
text_file = user_config_dir / "file.txt"
ph_file = user_config_dir / "ph.txt"
tds_file = user_config_dir / "tds.txt"
o2_file = user_config_dir / "o2.txt"
turbidity_file = user_config_dir / "turbidity.txt"

o2_calib_file = user_config_dir / "o2_calib.txt"
tds_calib_file = user_config_dir / "tds_calib.txt"
ph_calib_file = user_config_dir / "ph_calib.txt"
turbidity_calib_file = user_config_dir / "turbidity_calib.txt"
o2_calib_file2 = user_config_dir / "o2_calib2.txt"
tds_calib_file2 = user_config_dir / "tds_calib2.txt"
ph_calib_file2 = user_config_dir / "ph_calib2.txt"
turbidity_calib_file2 = user_config_dir / "turbidity_calib2.txt"

i2c = 0
ads = 0
o2_calib        =1.0
tds_calib       =1.0
ph_calib        =1.0
turbidity_calib =1.0
o2_calib2       =0.0
tds_calib2      =0.0
ph_calib2       =0.0
turbidity_calib2=0.0
o2_ble_value=0.0
class MM:
    def __init__(self) -> None:
        while True:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                ads = ADS.ADS1115(i2c)
                chan0 = AnalogIn(ads, ADS.P0)
                chan1 = AnalogIn(ads, ADS.P1)
                chan2 = AnalogIn(ads, ADS.P2)
                chan3 = AnalogIn(ads, ADS.P3)
                break
            except Exception:
                print("InitError")
        print("Init works")
        #asyncio.run(self.getSensors())
        self.time_since_boot = time.time()
        self.m2r_address = "http://localhost:6040/mavlink"
        self.request_timeout = 1.0
        self.mavlink2rest_package = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 9},
            "message": {"chunk_seq": 0,
            "id": 0,
            "severity": { "type": "MAV_SEVERITY_EMERGENCY" },
            "text": [ "H", "a", "l", "l", "o" ],
            "type": "STATUSTEXT"
            },
        }
    @unsync
    def send_statustext(self, message: str) -> None:
    
        lst = []

        for i in message:
            lst.append(i)
        self.mavlink2rest_package = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 1},
            "message": {"chunk_seq": 0,
            "id": 0,
            "severity": { "type": "MAV_SEVERITY_EMERGENCY" },
            "text": lst,
            "type": "STATUSTEXT" 
            },
        }
        asyncio.run(self.send_mavlink_message(1))
    def send_ph(self, ph: float) -> None:
        self.mavlink2rest_package = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 1},
            "message": {"time_usec": ((time.time() - self.time_since_boot) * 1000),
            "ph": float(ph),
            "type": "SENSOR_PH"
            },
        }
        asyncio.run(self.send_mavlink_message(1))
        #"message": { "time_usec": 1667512936,
    def send_sensors_to_mavlink(self, o2_value: float, tds_value: float, ph_value: float, turbidity_value: float) -> None:
        #(o2_value,tds_value,ph_value,turbidity_value)
        self.mavlink2rest_package = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 1},
            "message": { "time_usec": int(((time.time() - self.time_since_boot) * 1000)),
            "type": "GPS2_RAW",			
            "fix_type": { "type": "GPS_FIX_TYPE_3D_FIX" },
            "lat": int(o2_value*10000),
            "lon": int(tds_value*10000),
            "alt": int(ph_value*10000),
            "eph": int(turbidity_value*10000),
            "epv": 0,
            "vel": 0,
            "cog": 0,
            "satellites_visible": 0,
            "dgps_numch": 0,
            "dgps_age": 99999
            },
        }
        #print(int(((time.time() - self.time_since_boot) * 1000)))
        asyncio.run(self.send_mavlink_message(1))

    def getO2(self) -> None:
        while True:
                try:
                    url = 'http://192.168.42.123/events'
                    response = requests.get(url, stream=True)
                    for line in response.iter_lines():
                        if line:
                            line = line.decode('utf-8')
                            if '"id":' in line:
                                data_dict = json.loads(line[6:])
                                #if data_dict['name'] == 'BLE-9100 O2':
                                if data_dict['name'] == 'BLE-9100 Temperature':
                                    if data_dict['value'] is not None:
                                        print('BLE-9100 O2 value:', data_dict['value']) 
                except Exception as e:

                    print("Eine Fehler bei O2 ist aufgetreten:", str(e))
    
    def read_channel(bus, ads_address, channel,self) -> None:
            
        config = [0x84, 0x83 | (channel << 4)]
        bus.write_i2c_block_data(ads_address, 0x01, config)

        time.sleep(0.1)


        data = bus.read_i2c_block_data(ads_address, 0x00, 2)


        value = data[0] * 256 + data[1]


        if value > 0x7FF:
            value -= 0x1000
        
        voltage = value * 4.096 / 2047

        return voltage
                        
    def getSensors(self) -> None:
        
        while True:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                ads = ADS.ADS1115(i2c)
                chan0 = AnalogIn(ads, ADS.P0)
                chan1 = AnalogIn(ads, ADS.P1)
                chan2 = AnalogIn(ads, ADS.P2)
                chan3 = AnalogIn(ads, ADS.P3)
                break
            except Exception as e:
                print("InitError" +  e)
        print("readed Values")        
        list=[],[],[],[] 
        while True:
            time.sleep(0.1)
            #print("written")
            voltage=[0.0,0.0,0.0,0.0]
            list_o2=list[0]
            list_tds=list[1]
            list_ph=list[2]
            list_turbidity=list[3]
            try:
                #Auslesen der Analogen Calib Werte
                f=open(o2_calib_file, "r")
                o2_calib = float(f.read())
                f=open(ph_calib_file, "r")
                ph_calib = float(f.read())
                f=open(tds_calib_file, "r")
                tds_calib = float(f.read())
                f=open(turbidity_calib_file, "r")
                turbidity_calib = float(f.read())
                
                f=open(o2_calib_file2, "r")
                o2_calib2 = float(f.read())
                f=open(ph_calib_file2, "r")
                ph_calib2 = float(f.read())
                f=open(tds_calib_file2, "r")
                tds_calib2 = float(f.read())
                f=open(turbidity_calib_file2, "r")
                turbidity_calib2 = float(f.read())
                

                list_ph.append((chan0.voltage+ph_calib2)*ph_calib)

                list_turbidity.append((chan1.voltage+turbidity_calib2)*turbidity_calib)    

                
                list_o2.append((chan2.voltage+o2_calib2)*o2_calib)
                #print(statistics.mean(list_ph))
                list_tds.append((chan3.voltage+tds_calib2)*tds_calib)

                #list_o2.append((voltage[0]+o2_calib2)*o2_calib)
                #list_tds.append((voltage[1]+tds_calib2)*tds_calib)
                #list_ph.append((voltage[2]+ph_calib2)*ph_calib)
                #list_turbidity.append((voltage[3]+turbidity_calib2)*turbidity_calib)
                #print(len(list[0])," ",end=" ")
                #print(chan2.voltage)
                #print(chan3.voltage)
                if len(list[0]) > 9:
                    #print(ph_calib2)
                    #print(ph_calib)
                     #print(" ")
                    for z in range(4):
                         resall = statistics.variance(list[z])
                         for y in range(3):
                           for x in range(len(list[z])):
                             listneu=list[z].copy()
                             del listneu[x]
                             res = statistics.variance(listneu)
                             if resall > res:
                               y=x
                           del list[z][y]
                           resall = statistics.variance(list[z])
                           #print(list[z]) 
                           #print("The variance of list is : " + str(resall))
                         list[z][0] = statistics.mean(list[z])
                            
                    #print(list)
                    #Schreiben der Analogen Werte
                    f=open(o2_file, "w")
                    f.write(str(round(list[0][0],3)))
                    f=open(tds_file, "w")
                    f.write(str(round(list[1][0],3)))
                    f=open(ph_file, "w")
                    f.write(str(round(list[2][0]+(list[3][0]*1.70),3)))
                    f=open(turbidity_file, "w")
                    f.write(str(round(list[3][0],3)))

                    #Logoutput
                    print(str(round(list[0][0],3)), str(round(list[1][0],3)), str(round(list[2][0]+(list[3][0]*2),3)), str(round(list[3][0],3)))                   
                    self.send_sensors_to_mavlink(round(list[0][0],3),round(list[1][0],3),round(list[2][0],3),o2_ble_value)
                    list=[],[],[],[] 
                    #print("Daten geschrieben")
                    #self.send_statustext("TEST")
            except Exception as e:
                #Ausgeben des Fehlers
                x=1 #
                print("ValueError:" + e)
                list=[],[],[],[] 
    
    
    
            
    def getSensorsService(self) -> None:
        self.getSensors()
        #t = Thread(target=self.getSensors, args=())
        #t= Thread(target=, args())
        #t.run()
        
        
    
    async def send_mavlink_message(self, port: int) -> None:
        async with aiohttp.ClientSession() as session:
            try:
                async with session.post(
                     self.m2r_address, data=json.dumps(self.mavlink2rest_package), timeout=self.request_timeout
                ) as response:
                    print(f"Received status code of {response.status}.")
                    if not response.status == 200:
                        print(f"Received status code of {response.status}.")
            except asyncio.exceptions.TimeoutError as error:
                print(f"Request timed out after {request_timeout} second.") 
                await asyncio.sleep(1)
                

class TextData(BaseModel):
    data: str


SERVICE_NAME = "Sensor2Mavlink"

app = FastAPI(
    title="Sensor2Mavlink",
    description="API to send Sensors to Mavlink",
)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(14, GPIO.OUT)
GPIO.output(16, True)
GPIO.output(20, True)
GPIO.output(14, False)
test=MM()
#test.getSensorsService()
print("written")

thread_Sensors = threading.Thread(target=test.getSensors, args=(), daemon=True)
thread_Sensors.start()
thread_o2 = threading.Thread(target=test.getO2, args=(), daemon=True)
thread_o2.start()
print("Thread strated")

#p = Process(target=test.getSensorsService, args=())
#p.start()
#p.join()
#start_new_thread(test.getSensorsService,())
#s.deamon = True
#t= Thread(target=, args())



#test.getSensorsService()
#test.send_statustext(str("FEHLER IM SENSOR"))
servos = {}



logger.info(f"Starting {SERVICE_NAME}!")
logger.info(f"Text file in use: {text_file}")
logger.info(f"PH file in use: {ph_file}")
logger.info(f"TDS file in use: {tds_file}")
logger.info(f"O2 file in use: {o2_file}")
logger.info(f"Turbidity file in use: {turbidity_file}")

logger.info(f"O2_Calib file in use: {o2_calib_file}")
logger.info(f"TDS_Calib file in use: {tds_calib_file}")
logger.info(f"PH_Calib file in use: {ph_calib_file}")
logger.info(f"Turbidity_Calib file in use: {turbidity_calib_file}")

logger.info(f"O2_Calib2 file in use: {o2_calib_file2}")
logger.info(f"TDS_Calib2 file in use: {tds_calib_file2}")
logger.info(f"PH_Calib2 file in use: {ph_calib_file2}")
logger.info(f"Turbidity_Calib2 file in use: {turbidity_calib_file2}")



@app.get("/load_calib_ph", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_ph() -> Any:
    data = ""
    
    if ph_calib_file.exists():
        with open(ph_calib_file, "r") as f:
            data = f.read()
    else:
        with open(ph_calib_file, 'w') as f:
            f.write("0.0")
        data = 1.0
    logger.info(f"Load PH calib: {data}")
    return data
    
@app.get("/load_calib_tds", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_tds() -> Any:
    data = ""
    if tds_calib_file.exists():
        with open(tds_calib_file, "r") as f:
            data = f.read()
    else:
        with open(tds_calib_file, 'w') as f:
            f.write("0.0")
        data = 1.0
    logger.info(f"Load TDS calib: {data}")
    return data
    
@app.get("/load_calib_o2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_o2() -> Any:
    data = ""
    if o2_calib_file.exists():
        with open(o2_calib_file, "r") as f:
            data = f.read()
    else:
        with open(o2_calib_file, 'w') as f:
            f.write("0.0")
        data = 1.0
    logger.info(f"Load O2 calib: {data}")
    return data
    
@app.get("/load_calib_turbidity", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_turbidity() -> Any:
    data = ""
    if turbidity_calib_file.exists():
        with open(turbidity_calib_file, "r") as f:
            data = f.read()
    else:
        with open(turbidity_calib_file, 'w') as f:
            f.write("0.0")
        data = 1.0
    logger.info(f"Load turbidity calib: {data}")
    return data
    
    








@app.get("/load_calib_ph2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_ph2() -> Any:
    data = ""
    if ph_calib_file2.exists():
        with open(ph_calib_file2, "r") as f:
            data = f.read()
            logger.info(f"ph_calib_file2 file in use: {data}")
    else:
        with open(ph_calib_file2, 'w') as f:
            f.write("0.0")
        data = 0.0
    logger.info(f"Load PH calib2: {data}")
    return data
    
@app.get("/load_calib_tds2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_tds2() -> Any:
    data = ""
    if tds_calib_file2.exists():
        with open(tds_calib_file2, "r") as f:
            data = f.read()
    else:
        with open(tds_calib_file2, 'w') as f:
            f.write("0.0")
        data = 0.0
    logger.info(f"Load TDS calib2: {data}")
    return data
    
@app.get("/load_calib_o22", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_o22() -> Any:
    data = ""
    if o2_calib_file2.exists():
        with open(o2_calib_file2, "r") as f:
            data = f.read()
    else:
        with open(o2_calib_file2, 'w') as f:
            f.write("0.0")
        data = 0.0
    logger.info(f"Load O2 calib2: {data}")
    return data
    
@app.get("/load_calib_turbidity2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_calib_turbidity2() -> Any:
    data = ""
    if turbidity_calib_file2.exists():
        with open(turbidity_calib_file2, "r") as f:
            data = f.read()
    else:
        with open(turbidity_calib_file2, 'w') as f:
            f.write("0.0")
        data = 0.0
    logger.info(f"Load turbidity calib2: {data}")
    return data    
    
    
    



@app.get("/load_ph", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_ph() -> Any:
    data = ""
    if ph_file.exists():
        with open(ph_file, "r") as f:
            data = f.read()
    logger.info(f"Load PH: {data}")
    return data
    
@app.get("/load_tds", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_tds() -> Any:
    data = ""
    if tds_file.exists():
        with open(tds_file, "r") as f:
            data = f.read()
    logger.info(f"Load TDS: {data}")
    return data
    
@app.get("/load_o2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_o2() -> Any:
    data = ""
    if o2_file.exists():
        with open(o2_file, "r") as f:
            data = f.read()
    logger.info(f"Load O2: {data}")
    return data

@app.get("/load_Boje_time", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_Boje_time() -> Any:
    data=""
    return data

@app.get("/load_Boot_time", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_Boot_time() -> Any:
    x=datetime.datetime.now().strftime("%H:%M:%S  on  %d.%m.%Y")
    return str(x)



@app.get("/load_turbidity", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_turbidity() -> Any:
    data = ""
    if turbidity_file.exists():
        with open(turbidity_file, "r") as f:
            data = f.read()
    logger.info(f"Load turbidity: {data}")
    return data

@app.post("/save", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_data(data: TextData) -> Any:
    with open(text_file, "w") as f:
        f.write(data.data)

@app.get("/load", status_code=status.HTTP_200_OK)
@version(1, 0)
async def load_data() -> Any:
    data = ""
    if text_file.exists():
        with open(text_file, "r") as f:
            data = f.read()
    return data




@app.post("/save_calib_tds", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_tds_calib_file(data: TextData) -> Any:
    with open(tds_calib_file, "w") as f:
        f.write(data.data)

@app.post("/save_calib_ph", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_ph_calib_file(data: TextData) -> Any:
    with open(ph_calib_file, "w") as f:
        f.write(data.data)


@app.post("/save_calib_o2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_o2_calib_file(data: TextData) -> Any:
    with open(o2_calib_file, "w") as f:
        f.write(data.data)






@app.post("/save_calib_turbidity2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_turbidity_calib_file2(data: TextData) -> Any:
    with open(turbidity_calib_file2, "w") as f:
        logger.info(f"Safe TurCal2 {data}!")
        f.write(data.data)

@app.post("/save_calib_tds2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_tds_calib_file2(data: TextData) -> Any:
    with open(tds_calib_file2, "w") as f:
        f.write(data.data)

@app.post("/save_calib_ph2", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_ph_calib_file2(data: TextData) -> Any:
    with open(ph_calib_file2, "w") as f:
        f.write(data.data)


@app.post("/save_calib_o22", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_o2_calib_file2(data: TextData) -> Any:
    with open(o2_calib_file2, "w") as f:
        f.write(data.data)

@app.post("/setChamber", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setChamber(chamber: TextData) -> Any:
    print("Entnehme Probe aus der Kammer ",chamber.data)
    if int(chamber.data) == 1:
        test.send_statustext("Probe Eins wird genommen")
        GPIO.output(16, False)
        time.sleep(3)
        GPIO.output(16, True)
    if int(chamber.data) == 2:
        test.send_statustext("Probe Zwei wird genommen")
        GPIO.output(20, False)
        GPIO.output(16, False)
        time.sleep(3)
        GPIO.output(20, True)
        GPIO.output(16, True)
    return "ok"

@app.post("/setTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setTime() -> Any:
    print("Zeit synchronisieren")
    return "ok"


@app.post("/setGoPro", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setGoPro(chamber: TextData) -> Any:
    print("Entnehme Probe aus der Kammer ",chamber.data)
    if int(chamber.data) == 1:
        GPIO.output(14, True)
        test.send_statustext("GoPros An")
    if int(chamber.data) == 0:
        GPIO.output(14, False)
        test.send_statustext("GoPros Aus")
    return "ok"

@app.post("/save_calib_turbidity", status_code=status.HTTP_200_OK)
@version(1, 0)
async def save_turbidity_calib_file(data: TextData) -> Any:
    with open(turbidity_calib_file, "w") as f:
        logger.info(f"Safe TurCal {data}!")
        f.write(data.data)


#@app.post("/setServo", status_code=status.HTTP_200_OK)
#@version(1, 0)
#async def set_servo(data: TextData) -> Any:
#    print(data)  
    


@app.post("/setServo", status_code=status.HTTP_200_OK)
@version(1, 0)
async def set_servo(pin: int) -> Any:
    print("PIN:",pin)
    return "ok"

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"

if __name__ == "__main__":
    # Running uvicorn with log disabled so loguru can handle it
    uvicorn.run(app, host="0.0.0.0", port=8120, log_config=None)
