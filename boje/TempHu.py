from htu21 import HTU21 
htu = HTU21()
print("Temp")
print(htu.read_temperature())
print("Feuchte")
print(htu.read_humidity())
