#! /usr/bin/env python3
from pathlib import Path

import uvicorn
from loguru import logger
from typing import Any

class TextData(BaseModel):
    data: str


SERVICE_NAME = "ExampleExtension5"

app = FastAPI(
    title="Example Extension r API",
    description="API for an example extension that saves/loads data as files.",
)

logger.info(f"Starting {SERVICE_NAME}!")

servos = {}

@app.post("/setServo", status_code=status.HTTP_200_OK)
@version(1, 0)
async def set_servo(pin: int, pwm: float) -> Any:
    if pin < 0 or pwm < -1 or pwm > 1:
        raise HTTPException("Invalid values")
    if pin not in servos:
        servos[pin] = Servo(pin)
    servos[pin].value = pwm
    return "ok"

app = VersionedFastAPI(app, version="1.0.0", prefix_format="/v{major}.{minor}", enable_latest=True)

app.mount("/", StaticFiles(directory="static",html = True), name="static")

@app.get("/", response_class=FileResponse)
async def root() -> Any:
        return "index.html"

if __name__ == "__main__":
    # Running uvicorn with log disabled so loguru can handle it
    uvicorn.run(app, host="0.0.0.0", port=6789, log_config=None)
