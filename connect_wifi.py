import network
from time import sleep
def disconnect():
    station = network.WLAN(network.STA_IF)
    station.active(True)
    station.disconnect()
def connectTo(ssid, password):
    disconnect()
    station = network.WLAN(network.STA_IF)
    station.active(True)
    station.scan()
    i = 20
    while (i>0) and (station.isconnected() == False):
        station.connect(ssid, password)
        print("connecting to " + ssid + "   "+str(i))
        sleep(0.5)
        i=i-1
        
    if station.isconnected() == True:
        print("connected to network " + ssid)
        ip = station.ifconfig()
        print("your IP: " +str(ip) )
    else:
        print("we can't connect to network. please check!")


connectTo("Tên wifi", "mật khẩu")
