import asyncio
from bleak import BleakClient
import time

address = "C0:00:00:00:4E:54"
MODEL_NBR_UUID = "0000180D-0000-1000-8000-00805F9B34FB"
Battery_Service = "00002a19-0000-1000-8000-00805f9b34fb"
Vendor_specific = "0000ff10-0000-1000-8000-00805f9b34fb"

"""
00001800-0000-1000-8000-00805f9b34fb (Handle: 1): Generic Access Profile
00001801-0000-1000-8000-00805f9b34fb (Handle: 8): Generic Attribute Profile
0000180a-0000-1000-8000-00805f9b34fb (Handle: 12): Device Information
0000180f-0000-1000-8000-00805f9b34fb (Handle: 31): Battery Service
0000ff01-0000-1000-8000-00805f9b34fb (Handle: 36): Vendor specific
"""

class Messure(object):

    def __init__(self, frame : str) -> None:

        self.frame = frame
        self.packet = self.decode(frame)
        self.data = self.packet[0]
        self.constant = self.packet[1]
        self.product_name_code = self.packet[2]
        self.hold_reading = self.packet[17] >> 4
        self.backlight_status = (self.packet[17] & 0x0F) >> 3
        self.battery = self.decode_position(15) 
        self.prozent = (self.decode_position(5)/10)
        self.mgl = self.decode_position(3)/100                                       # Das ist wert in mg/L
        self.temperature = self.decode_position(13)/10 #--> das auch
        
       

    def decode(self, byte_frame : bytes ):

        frame_array = [int(x) for x in byte_frame]
        size = len(frame_array)
        for i in range(size-1, 0 , -1):
            tmp=frame_array[i]
            hibit1=(tmp&0x55)<<1
            lobit1=(tmp&0xAA)>>1
            tmp=frame_array[i-1]
            hibit=(tmp&0x55)<<1
            lobit=(tmp&0xAA)>>1
            frame_array[i]=0xff -(hibit1|lobit)
            frame_array[i-1]= 0xff -(hibit|lobit1)
        return frame_array
        
    def reverse_bytes(self, bytes : list):
        return (bytes[0] << 8) + bytes[1]
        
    def decode_position(self,idx):
        return self.reverse_bytes(self.packet[idx:idx+2])
        
    def show_values(self):
        return_string = f"hold=[{self.hold_reading}/backlight={self.backlight_status}/Bat={self.battery}] Prozent={self.prozent:4}%  mg/L={self.mgl:4}   Temperature(C)={self.temperature:4} "
        return return_string

   

async def main(address):
    async with BleakClient(address) as client:
        print(f"Connected: {client.is_connected}")
        paired = await client.pair(protection_level=2)
        print(f"Paired: {paired}")
        while True:

            model_number = await client.read_gatt_char(Vendor_specific) 
            print(Messure(model_number).show_values())
            time.sleep(1)

asyncio.run(main(address))