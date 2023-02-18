#! /usr/bin/env python3
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

    
class MM:
    def StartServer(self) -> None: 
        uvicorn.run(app, host="0.0.0.0", port=8120, log_config=None)



def main():

    Server=MM()
    print("written")

    thread_Server = threading.Thread(target=Server.StartServer, args=(), daemon=True)
    thread_Server.start()

main()
uvicorn.run(app, host="0.0.0.0", port=8120, log_config=None)