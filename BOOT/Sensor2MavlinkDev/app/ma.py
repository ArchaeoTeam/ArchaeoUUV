import asyncio
import json
import os
import time
from typing import Any, Dict, Optional
import sys

import aiohttp

#from commonwealth.mavlink_comm.MavlinkComm import MavlinkMessenger
#from loguru import logger


class MM:
    def __init__(self) -> None:

        self.time_since_boot = time.time()
        self.m2r_address = "http://localhost:6040/mavlink"
        self.request_timeout = 1.0
        self.mavlink2rest_package = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 9},
            "message": {"chunk_seq": 0,
            "id": 0,
            "severity": { "type": "MAV_SEVERITY_EMERGENCY" },
            "text": [ "H", "a", "l", "l", "o", " ", "D", "A", "V", "I", "D", " ", " ", " ", " ", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000", "\u0000" ],
            "type": "STATUSTEXT"
            },
        }
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
test=MM()
test.send_statustext(str(sys.argv[1]))
