import os, datetime, time
import uvicorn
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse, FileResponse
from fastapi_versioning import VersionedFastAPI, version
from pydantic import BaseModel
from typing import Any, Dict, Optional

import position_correction as pc

class TextData(BaseModel):
    data: str
    
class BoolData(BaseModel):
    data: bool

app = FastAPI(
    title="GPS_Calc",
    description="GPS_Calc",
)

######### GETTER
@app.get("/GPSLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.boje_position.lat

@app.get("/GPSLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.boje_position.lon

@app.get("/CorrectedLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.corrected_position.lat

@app.get("/CorrectedLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.corrected_position.lon

@app.get("/correction_offset_lon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.correction_offset_utm.lon

@app.get("/correction_offset_lat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.correction_offset_utm.lat

@app.get("/CorrectionPossible", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.correction_possible

@app.get("/DGPSpossible", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.rtk_possible

# Telemetry Data
@app.get("/Depth", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.depth

@app.get("/Lenght", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.distance

@app.get("/Correction_Offset", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.correction_offset

@app.get("/Direction", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.direction

@app.get("/Compass", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.rov_compass

@app.get("/Accuracy", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.accuracy


@app.get("/getTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=datetime.datetime.now().strftime("%H:%M:%S  on  %d.%m.%Y")
   return str(x)

@app.get("/getTemp", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(pc.htu.read_temperature(),1)
   #print("T=",x)
   return str(x)

@app.get("/getHum", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(pc.htu.read_humidity(),1)
   #print("H=",x)
   return str(x)

enableRTK = pc.enable_RTK
@app.get("/getDGPSenabled", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_RTK
   return str(enable_RTK)

enable_correction = pc.enable_correction
@app.get("/getCorrectionEnabled", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_correction
   return str(enable_correction)


@app.get("/getFixLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   return pc.fix_point.lat

@app.get("/getFixLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   return pc.fix_point.lon

@app.get("/getLogs", status_code=status.HTTP_200_OK)
@version(1, 0)
async def getLogs() -> Any:
   # TODO: return lsit of all files in ./logs/ seperated by \n
   # time.strftime("%Y%m%d")
   files = os.listdir("./logs/"+time.strftime("%Y%m%d"))
   log_files_str = "\n".join(files)
   return log_files_str

######### SETTER
@app.post("/setFixLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(FixLat: TextData) -> Any:
    pc.fix_point.lat=FixLat.data
    print("FIXLAT=",pc.fix_point.lat)
    return "ok"

@app.post("/setFixLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(FixLon: TextData) -> Any:
    pc.fix_point.lon=str(FixLon.data)
    print("FIXLON=",FIXLON)
    return "ok"


@app.post("/enableDGPS", status_code=status.HTTP_200_OK)
@version(1, 0)
async def set(setDGPS: BoolData) -> Any:
   global enable_RTK
   enable_RTK = setDGPS.data
   print("###DGPS  ",enable_RTK)
   return "ok"

#TODO: add the option to start a new log

@app.post("/enableCalc", status_code=status.HTTP_200_OK)
@version(1, 0)
async def enableCalc(setCorrection: BoolData) -> Any:
    global enable_correction
    enable_correction = setCorrection.data
    print("###Correction  ",enable_correction)
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

@app.post("/newLog", status_code=status.HTTP_200_OK)
@version(1, 0)
async def newLog() -> Any:
    with open("./logs/"+time.strftime("%Y%m%d")+"/Experiment_times.txt", "a") as f:
        dt = time.strftime("%Y%m%d-%H%M%S")
        # Write the datetime to the file
        f.write(f"{dt}\n")

    pc.startNewLog()
    print("### Starting new log")
    return "ok"

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"

# Start Webserver
uvicorn.run(app, host="0.0.0.0", port=88, log_config=None)
