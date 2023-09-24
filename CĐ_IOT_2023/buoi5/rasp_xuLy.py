import json
import paho.mqtt.client as mqtt
from time import strftime
import json
# from grove.display.jhd1802 import JHD1802 as LCD
# from grove.grove_relay import GroveRelay
# from upm import pyupm_buzzer as bz
# from gpiozero import LED
# from seeed_dht import DHT

# khai bao chan
# led=LED(22)
# relay=GroveRelay(5)
#lcd=LCD()
# buzzer = LED(16)
# dht = DHT("11",18)


# tao bien global de luu cac gia tri
mode = 0
led_state = 0
buzzer_state = 0
relay_state = 0
humi = 90
temp = 25

def dieu_khien():
    if mode == 1: # che do manual
    # kiem tra led
        print("mode manual")
        if led_state == 1:
            # led.on()
            print("led on")
        elif led_state == 0:
            # led.off()
            print("led off")
        # kiem tra buzzer 
        if buzzer_state == 1:
            print("buzzer on")
            # buzzer.on()
        elif buzzer_state == 0:
            # buzzer.off()
            print("buzzer off")
            # kiem tra relay
        if relay_state == 1:
            # relay.on()
            print("relay on")
        elif relay_state == 0:
            # relay.off()
            print("relay off")

    elif mode == 0: 
        print("mode auto")
        h = strftime("%H").split(':')
        h = int(h[0])
        if 18 < h and h < 20:
            # led.on()
            print("led on")
        else:
            # led.off()
            print("led off")
                
        if temp > 37:
            # buzzer.on()
            print("buzzer on")
        elif temp < 31:
            # buzzer.off()
            print("buzzer off")
            
        if humi > 90:
            # relay.on()
            print("relay on")
        elif humi < 60:
            # relay.off()
            print("relay off")

def on_connect(client, userdata,flags,rc):
    channel_ID = "2272720"
    print("Connected with result code {}".format(rc))
    client.subscribe('channels/'+channel_ID+'/subscribe')
    
def on_disconnect(client,userdata,rc):
    print("Disconnected from Broker")

def on_message(client, userdata, message):
    print('debug')
    global mode, led_state, relay_state, buzzer_state, humi,temp
    fields = message.payload.decode()
    fields = json.loads(str(fields))
    
    if fields['field3'] != None:  # check mode
        mode = int(fields['field3'])
        print('mode: ', mode)
    if fields['field4'] != None:  # check led_state
        led_state = int(fields['field4'])
        print('led state: ', led_state)
    if fields['field5'] != None:  # check buzzer_state
        buzzer_state = int(fields['field5'])
        print('buzzer state: ', buzzer_state)
    if fields['field6'] != None:  # check relay_state
        relay_state = int(fields['field6'])
        print('relay state: ', relay_state)
    if fields['field1'] != None:  # check humi
        humi = int(fields['field1'])
        print('humi: ', humi)
    if fields['field2'] != None:  # check temp
        temp = int(fields['field2'])
        print('temp: ', temp)
    
    print(mode, led_state, relay_state, buzzer_state, humi, temp)
    dieu_khien()
    print('=========================================')
# ========================================================================
client_id = 'ETYpPQIvAQ8jCjI1By0AGRc'
client = mqtt.Client(client_id)

# gan cac chuong trinh con
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.username_pw_set(username='ETYpPQIvAQ8jCjI1By0AGRc', password= '+YvXrKasAdP1NW/zKTvsjv8U')
client.connect("mqtt3.thingspeak.com", 1883,60)
client.loop_forever()
        
 