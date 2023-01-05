from dronekit import connect
import time, socket

connection_boje = 'localhost:14550'
#   Open socket to GCS
BOOT_IP = "192.168.2.1"
BOOT_PORT = 27123
sock_boot = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP


boje = connect(connection_boje,"baud=57600")#, wait_ready=False)

print("Sending voltage to "+ BOOT_IP +":"+ str(BOOT_PORT))

while(boje):
    time.sleep(1)    
    sock_boot.sendto(bytes(str(boje.battery.voltage), encoding='utf8'), (BOOT_IP, BOOT_PORT))
    print(boje.battery)