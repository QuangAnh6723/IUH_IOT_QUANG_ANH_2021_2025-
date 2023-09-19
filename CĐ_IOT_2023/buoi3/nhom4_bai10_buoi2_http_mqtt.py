from time import sleep
from urllib import request
from seeed_dht import DHT
from grove.display.jhd1802 import JHD18
from grove.grove_light_sensor_v1_2 import GroveLightSensor
from grove.grove_ultrasonic_ranger import GroveUltrasonicRanger
import paho.mqtt.client as mqtt

client = mqtt.Client("HRQcAz0pFjMVDAQQHAMAEi4")
client.username_pw_set(username="HRQcAz0pFjMVDAQQHAMAEi4", password= "2Z0hp8mIV23JTiG2HI7XdO0i")
client.connect("mqtt3.thingspeak.com", 1883,60)

lcd = JHD1802()
sensor = DHT('11', 5)
sensorkc = GroveUltrasonicRanger(16)
sensorlight = GroveLightSensor(0)

def post_http(data):
    url = "https://api.thingspeak.com/update?api_key=B94TVA2P8ECAETG7"
    url += data
    request.urlopen(url)
    # print("http")

def mqtt_pub(data):
    channel_ID = "2265660"
    data_pub = data + "&status=MQTTPUBLISH"
    client.publish("channels/%s/publish" %(channel_ID), data_pub)
    # print('mqtt')

while True:
    
    humi, temp = sensor.read()
    dis = int(sensorkc.get_distance())
    light = sensorlight.light
    
    lcd.setCursor(0, 0)
    lcd.write('{0:2}C'.format(temp))
    lcd.setCursor(0, 6)
    lcd.write('{0:5}%'.format(humi))
    lcd.setCursor(1, 0)
    lcd.write('{0:2}'.format(light))
    lcd.setCursor(1, 6)
    lcd.write('{0:2} cm'.format(dis))

    # tao chuoi &field1=gt1&field2=gt2&field3=gt3&field4=gt4
    post = "&field1="
    post += str(humi)
    post += "&field2="
    post += str(temp)
    post += "&field3="
    post += str(light)
    post += "&field4="
    post += str(dis)
    
    post_http(post)
    mqtt_pub(post)
    sleep(20)



