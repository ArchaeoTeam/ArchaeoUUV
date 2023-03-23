import datetime
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

app = FastAPI(
    title="GPS_Calc",
    description="GPS_Calc",
)

# GPS Data
@app.get("/GPSLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.boje_position.lat

@app.get("/GPSLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.boje_position.lon

@app.get("/cGPSPosLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.corrected_position.lat

@app.get("/cGPSPosLon", status_code=status.HTTP_200_OK)
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


# correction Data
@app.get("/Depth", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.depth

@app.get("/Lenght", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.distance

@app.get("/Correction", status_code=status.HTTP_200_OK)
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


#DGPS Data
@app.get("/DGPSfixLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   #print("data")
   return pc.fix_point.lat

@app.get("/DGPSfixLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.fix_point.lon

@app.get("/DGPSposLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.dgps_nmea.latitude

@app.get("/DGPSposLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.dgps_nmea.longitude

@app.get("/DGPScorrX", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.DGPSpoint.GetX()

@app.get("/DGPScorrY", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
    return pc.DGPSpoint.GetY()


@app.get("/GetTime", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=datetime.datetime.now().strftime("%H:%M:%S  on  %d.%m.%Y")
   return str(x)


@app.get("/GetTemp", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(pc.htu.read_temperature(),1)
   #print("T=",x)
   return str(x)

@app.get("/GetHum", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   x=round(pc.htu.read_humidity(),1)
   #print("H=",x)
   return str(x)

enableRTK = pc.enable_RTK
@app.get("/GetDGPS", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_RTK
   return str(enable_RTK)

enable_correction = pc.enable_correction
@app.get("/Getcorr", status_code=status.HTTP_200_OK)
@version(1, 0)
async def loadData() -> Any:
   global enable_correction
   return str(enable_correction)


@app.post("/setDGPSLat", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(setDGPSLat: TextData) -> Any:
    pc.fix_point.lat=setDGPSLat.data
    print("FIXLAT=",pc.fix_point.lat)
    return "ok"

@app.post("/setDGPSLon", status_code=status.HTTP_200_OK)
@version(1, 0)
async def setData(setDGPSLon: TextData) -> Any:
    pc.fix_point.lon=str(setDGPSLon.data)
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

#TODO: add the option to start a new log

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

# Start Webserver
uvicorn.run(app, host="0.0.0.0", port=88, log_config=None)
