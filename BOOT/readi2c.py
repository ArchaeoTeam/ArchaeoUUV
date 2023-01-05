import board
import busio
import time
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import numpy as np
import datetime
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)


ph = AnalogIn(ads, ADS.P0)
leit = AnalogIn(ads, ADS.P1)
temp = AnalogIn(ads, ADS.P2)
tmpArr = []
counter = 0
temperature = 20

while(True):
    # time.sleep(0.004)
    time.sleep(1)
    ts = datetime.datetime.now()

    print("--------------------------\n"+str(ts))
    print("PHobj: " + str(ph.value), str(ph.voltage))
    print("Leit: " + str(leit.value), str(leit.voltage))
    print("Temp: " + str(temp.value), str(temp.voltage))
    
    #Convert voltage value to TDS value TODO: calibrieren, genauigkeit erhöhen
    Voltage = leit.voltage
    if(Voltage == 0.0):
        Voltage = 0.1

    if(leit.voltage > 0.0):
        tdsValue = (133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5
        print("\nTDS = " + str(tdsValue))

    if(ph.voltage > 0.0):
        phValue = 7 + ((2.5 - ph.voltage) / 0.18)
        print("PH-Value = "+ str(phValue)+"\n")
    