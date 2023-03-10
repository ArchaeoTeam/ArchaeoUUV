#!/usr/bin/env python3

import os
import ssl
import appdirs
from pathlib import Path
from setuptools import setup

if os.path.exists(appdirs.user_config_dir()):
   logger.info(f"Configfile gibt es schon")
else:
   os.mkdir(appdirs.user_config_dir())

files = ["file.txt", "ph.txt","tds.txt","o2.txt","turbidity.txt","o2_calib.txt","tds_calib.txt","ph_calib.txt","turbidity_calib.txt"]
user_config_dir = Path(appdirs.user_config_dir())
fruits = ["apple", "banana", "cherry"]
for x in files:
  text_file=user_config_dir /  x
  f=open(text_file, "w")
  f.write(str(1.0))


# Ignore ssl if it fails
if not os.environ.get("PYTHONHTTPSVERIFY", "") and getattr(ssl, "_create_unverified_context", None):
    ssl._create_default_https_context = ssl._create_unverified_context

setup(
    name="Sensor2Mavlink",
    version="0.1.0",
    description="Sensor 2 Mavlink",
    license="MIT",
    install_requires=[
        "appdirs == 1.4.4",
        "fastapi == 0.63.0",
        "fastapi-versioning == 0.9.1",
        "loguru == 0.5.3",
        "uvicorn == 0.13.4",
        "starlette==0.13.6",
        "aiofiles==0.8.0",
        "gpiozero==1.6.2",
        "unsync==1.4.0",
        "board==1.0",
        "aiohttp==3.8.3",
        "adafruit-blinka==8.11.0",
        "starlette==0.13.6",
        "aiofiles==0.8.0",
        "gpiozero==1.6.2",
    ],
)


