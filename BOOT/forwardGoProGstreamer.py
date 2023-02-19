import socket
import time
import requests
import json
import atexit
import sys
import psutil
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# Get the IP address of the "usb0" interface
usb_interface = psutil.net_if_addrs()["usb0"][0]
if usb_interface is None:
    print("GoPro not connected properly! No usb0 interface")
    sys.exit()

# Set the IP address of the GoPro to the IP address of the "usb0" interface, with the last octet set to 1
GOPRO_IP = usb_interface.address[:-1] + "1"
print("GoPro connected on " + GOPRO_IP)

# Set the URL for starting the webcam mode on the GoPro
START_URL = "http://" + GOPRO_IP + "/gp/gpWebcam/START?res=1080"

# Set the URL for stopping the webcam mode on the GoPro
STOP_URL = "http://" + GOPRO_IP + "/gp/gpWebcam/STOP"

# Set the target IP address and port for the RTP stream
try:
    args = sys.argv[1].split(':')

    RTP_IP = args[0]
    RTP_PORT = int(args[1])
except:
    print("Parameter needed in form of <IP:PORT>")
    sys.exit()

# Define the exit handler function
def exit_handler():
    print("\nStopping Webcam...\n")
    requests.get(STOP_URL)

# Register the exit handler
atexit.register(exit_handler)

# Initialize GStreamer
Gst.init(None)

# Create a pipeline for encoding and sending the RTP stream
pipeline = Gst.Pipeline()

# Create a udpsink for sending the RTP stream
udpsink = Gst.ElementFactory.make('udpsink', 'udpsink')
udpsink.set_property('host', RTP_IP)
udpsink.set_property('port', RTP_PORT)

# Create an rtph264pay element for adding RTP headers to the H.264 video packets
rtph264pay = Gst.ElementFactory.make('rtph264pay', 'rtph264pay')

# Create an appsrc element for receiving the video data from the GoPro
appsrc = Gst.ElementFactory.make('appsrc', 'appsrc')

# Add the elements to the pipeline
pipeline.add(appsrc)
pipeline.add(rtph264pay)
pipeline.add(udpsink)

# Link the elements in the pipeline
appsrc.link(rtph264pay)
rtph264pay.link(udpsink)

# Start the webcam mode on the GoPro
r = requests.get(START_URL)
r_json = json.loads(r.text)

# Check if the webcam mode was successfully started
if r_json["error"] == 0 and r_json["status"] == 2:
    print("\nGoPro Webcam mode started\n")
    print("Sending RTP stream to " + RTP_IP + ":" + str(RTP_PORT))

# Set up a callback function for the appsrc element
def new_sample(sink):
    # Create a buffer for the video data
    buffer = sink.emit('pull-sample')
    # Get the video data from the buffer
    video_data = buffer.get_data()
    # Push the video data to the appsrc element
    sink.emit('push-buffer', buffer)
    return True

# Set the "caps" property of the appsrc element to indicate the video format that is being received
appsrc.set_property('caps', Gst.Caps.from_string('video/x-h264,width=1920,height=1080,framerate=30/1'))

# Set the "block" property of the appsrc element to False to enable streaming mode
appsrc.set_property('block', False)

# Connect the "need-data" signal of the appsrc element to the callback function
appsrc.connect('need-data', new_sample)

# Start the pipeline
pipeline.set_state(Gst.State.PLAYING)

# Enter the main loop
loop = GObject.MainLoop()
loop.run()