

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

from pydantic import BaseModel
import os

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
#import adafruit_ads1x15.ads1115 as ADS
#from adafruit_ads1x15.analog_in import AnalogIn
#import RPi.GPIO as GPIO


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

class MM:
    def __init__(self) -> None:
        print("Init")#

    def update(self) -> None:
        print("Update")

                

class TextData(BaseModel):
    data: str


SERVICE_NAME = "Sensor2Mavlink"

app = FastAPI(
    title="Sensor2Mavlink",
    description="API to send Sensors to Mavlink",
)

test=MM()
print("written")

thread_Update = threading.Thread(target=test.update, args=(), daemon=True)
thread_Update.start()
print("Thread strated")


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
    x=datetime.datetime.now().strftime("%H:%M  on  %d.%m.%Y")
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
        GPIO.output(16, False)
        time.sleep(3)
        GPIO.output(16, True)
    if int(chamber.data) == 2:
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
    if int(chamber.data) == 0:
        GPIO.output(14, False)
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
