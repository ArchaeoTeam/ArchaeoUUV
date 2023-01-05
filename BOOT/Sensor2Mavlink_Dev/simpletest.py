# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
from pathlib import Path
import appdirs

import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

user_config_dir = Path(appdirs.user_config_dir())
text_file = user_config_dir / "file.txt"
ph_file = user_config_dir / "ph.txt"
tds_file = user_config_dir / "tds.txt"
o2_file = user_config_dir / "o2.txt"
turbidity_file = user_config_dir / "turbidity.txt"

o2_calibfile = user_config_dir / "o2_calib.txt"
tds_calibfile = user_config_dir / "tds_calib.txt"
ph_calibfile = user_config_dir / "ph_calib.txt"
turbidity_calibfile = user_config_dir / "turbidity_calib.txt"
i2c = 0
ads = 0
o2_calib       =1.0
tds_calib      =1.0
ph_calib       =1.0
turbidity_calib=1.0
# Create single-ended input on channel 0
while True:
    try:

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        chan0 = AnalogIn(ads, ADS.P0)
        chan1 = AnalogIn(ads, ADS.P1)
        chan2 = AnalogIn(ads, ADS.P2)
        chan3 = AnalogIn(ads, ADS.P3)
        break
    except Exception:
        print("InitError")
# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

print("{:>5}\t{:>5}".format("raw", "v"))

while True:
    time.sleep(0.2)

    try:
        #Öffnen der Calibfiles
        f=open(o2_calibfile, "r")
        o2_calib = float(f.read())
        f=open(ph_calibfile, "r")
        ph_calib = float(f.read())
        f=open(tds_calibfile, "r")
        tds_calib = float(f.read())
        f=open(turbidity_calibfile, "r")
        turbidity_calib = float(f.read())

        #Auslesen der Analogen Werte
        o2_value=chan0.voltage*o2_calib
        tds_value=chan1.voltage*tds_calib
        ph_value=chan2.voltage*ph_calib
        turbidity_value=chan3.voltage*turbidity_calib

        #Schreiben der Analogen Werte
        f=open(o2_file, "w")
        f.write(str(round(o2_value,3)))
        f=open(tds_file, "w")
        f.write(str(round(tds_value,3)))
        f=open(ph_file, "w")
        f.write(str(round(ph_value,3)))
        f=open(turbidity_file, "w")
        f.write(str(round(turbidity_value,3)))

        #Logoutput
        print("written")
        print("{:>5.3f}\t{:>5.3f}\t{:>5.3f}\t{:>5.3f}".format(o2_value, tds_value, ph_value, turbidity_value))

    except Exception:
        #Ausgeben des Fehlers
        x=1 #print("ValueError")
    

