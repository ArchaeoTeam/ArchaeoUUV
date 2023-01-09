import socket
import time
import requests
import json
import atexit
import sys
import psutil
import asyncio
import websockets

# Find the IP address of the GoPro's USB interface
usb_interface = psutil.net_if_addrs()["usb0"][0]
if usb_interface == None:
    print("GoPro not connected properly! No usb0 interface")
    sys.exit()

# Set the GoPro's IP address based on the USB interface's address
GOPRO_IP = usb_interface.address[:-1] + "1"
print("GoPro connected on " + GOPRO_IP)

# URL for starting the GoPro's webcam feature
START_URL = "http://" + GOPRO_IP + "/gp/gpWebcam/START?res=1080"

# URL for stopping the GoPro's webcam feature
STOP_URL = "http://" + GOPRO_IP + "/gp/gpWebcam/STOP"

# Register an exit handler to stop the GoPro's webcam feature when the script is interrupted or terminated
def exit_handler():
    print('\nStopping Webcam...\n')
    requests.get(STOP_URL)

atexit.register(exit_handler)

# Start the GoPro's webcam feature
r = requests.get(START_URL)
r_json = json.loads(r.text)

if r_json['error'] == 0 and r_json['status'] == 2:
    print("\nGoPro Webcam mode started\n")
else:
    print("Error starting GoPro webcam mode:", r_json)
    sys.exit()

# Set up a UDP server to receive video data from the GoPro
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 8554))

# Function to send video data over WebRTC
async def send_video_data(websocket):
    while True:
        data, addr = sock.recvfrom(2048)
        await websocket.send(data)

# Function to set up a WebRTC connection and start sending video data
async def start_webrtc(uri):
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print("WebRTC connection established!")
                # Forward UDP stream over WebRTC
                while True:
                    data, addr = sock.recvfrom(2048)
                    await websocket.send(data)
        except Exception as e:
            print(f"Error establishing WebRTC connection: {e}")
            print("Retrying in 5 seconds...")
            await asyncio.sleep(5)

# Set up the WebRTC URI based on the IP address and port number specified as command-line arguments
try:
    #args = sys.argv[1].split(':')

    WEBRTC_IP = "127.0.0.1" #args[0]
    WEBRTC_PORT = 8004 #int(args[1])
    WEBRTC_URI = f"ws://{WEBRTC_IP}:{WEBRTC_PORT}"
except:
    print("Parameter needed in form of <IP:PORT>")
    sys.exit()

print("Forwarding UDP stream to " + WEBRTC_IP + ":" + str(WEBRTC_PORT) + " over WebRTC\n")

# Start the WebRTC connection
asyncio.get_event_loop().run_until_complete(start_webrtc(WEBRTC_URI))