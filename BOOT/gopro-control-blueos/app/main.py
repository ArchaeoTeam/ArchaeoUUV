#! /usr/bin/env python3
from pathlib import Path

import uvicorn
from loguru import logger
from typing import Any


SERVICE_NAME = "GoProControl"

logger.info(f"Starting {SERVICE_NAME}!")

async def app() -> Any:
        return "index.html"

if __name__ == "__main__":
    # Running uvicorn with log disabled so loguru can handle it
    uvicorn.run("main:app", host="0.0.0.0", port=6789, log_config=None)
