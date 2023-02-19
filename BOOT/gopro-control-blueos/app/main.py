#! /usr/bin/env python3
from pathlib import Path
import sys
import uvicorn
import websockets
from websockets import WebSocketServerProtocol
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse, FileResponse
from fastapi_versioning import VersionedFastAPI, version
from loguru import logger
from typing import Any
from pydantic import BaseModel, Field


try:
    PORT = sys.argv[1]
except:
    print("Please provide a port number as parameter!")
    sys.exit()

SERVICE_NAME = "GoProControl"

app = FastAPI(
    title="Go Pro Preview and Control",
    description="This is a simple web app to preview and control a GoPro camera",
)

class WebSocketServerProtocolField(Field):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.type_ = "websocket"
        self.model_field_type = WebSocketServerProtocol

class WebsocketModel(BaseModel):
    websocket: WebSocketServerProtocolField

logger.info(f"Starting {SERVICE_NAME}!")

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.websocket("/stream")
async def websocket_endpoint(websocket: WebsocketModel):
    logger.info(f"Connect to ws://127.0.0.1: {str(PORT-1)}!")
    # Connect to the WebRTC server and receive the stream
    async with websockets.connect("ws://127.0.0.1:"+str(PORT-1)) as websocket_in:
        # Forward the stream to the client over the FastAPI websocket
        while True:
            data = await websocket_in.recv()
            await websocket.send(data)


@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"

if __name__ == "__main__":
    # Running uvicorn with log disabled so loguru can handle it
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_config=None)
