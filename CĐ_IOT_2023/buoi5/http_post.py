# from time import sleep, strftime
# import json
# from grove.display.jhd1802 import JHD1802 as LCD
# from time import sleep
# import json
# import paho.mqtt.client as mqtt
# from grove.display.jhd1802 import JHD1802 as LCD
# from seeed_dht import DHT
# ===================================
# khai bao thu vien
from time import sleep, strftime
from urllib import request
import random as rd

# khai bao thiet bi
# lcd=LCD()
# dht = DHT("11",18)

# khai bao channel 
channel_ID = "2272720"


def post_http():
    api_key = "27G90F1UEBXTY6IY"
    url = "https://api.thingspeak.com/update?api_key=%s&field1=%s&field2=%s" %(api_key,temp,humi)
    request.urlopen(url)
    r=request.urlopen(url)
    print("http send ok ")

while True:
    # humi, temp = dht.read()
    humi = rd.randint(60,100)
    temp = rd.randint(20,40)

    h,m,s = strftime("%H:%M:%S").split(":")
    print(strftime("%H:%M:%S"), end= " : ")
    # lcd.setCursor(0,0)
    # lcd.write("Time: {}:{}  ".format(h,m))
    post_http()
    sleep(15)
    
