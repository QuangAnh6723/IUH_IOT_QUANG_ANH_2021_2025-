import paho.mqtt.client as mqtt
import json
import time
from gpiozero import LED
from grove.display.jhd1802 import JHD1802

led = LED(26)
buzer = LED(18)
relay = LED(16)
lcd = JHD1802()

def on_connect(client, userdata, flags, rc):
    print("Connected with results code {}".format(rc))
    channel_ID = "2273178"
    client.subscribe("channels/%s/subscribe" % channel_ID)

def on_disconnect(client, userdata, rc):
    print("disconnect from broker")
    
def on_message(client, userdata, message):
    data_fields = message.payload.decode()
    data_fields = json.loads(data_fields)

    rand = int(data_fields["field3"])
    humi = int(data_fields["field2"])
    temp = int(data_fields["field1"])

    print("lcd: \nNhiet do: %s - Do am: %s - Rand: %s" %(temp,humi,rand))
    lcd.setCursor(0,0)
    lcd.write('Temp: {}'.format(temp))
    lcd.setCursor(0,9)
    lcd.write('Hum: {}'.format(humi))
    lcd.setCursor(1,0)
    lcd.write('Rand: {}'.format(rand))

    if rand > 50:
        led.on()
    elif rand < 50:
        led.off()
    else:
        print("vao rand")
    
    if temp > 37:
        buzer.on()
    elif temp < 31:
        buzer.off()
    else:
        print("ko doi trang thai")
    
    if humi > 90:
        relay.on()
    elif humi < 80:
        relay.off()
    else:
        print("ko doi trang thai relay")

client_id = 'NjUpFAoPLAIEMjwoFwotJTA'
client = mqtt.Client(client_id)
# gan cac chuong trinh con
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.username_pw_set(username='NjUpFAoPLAIEMjwoFwotJTA', password= 'uxiAQcEHk7LlSuIu7jm3cmxo')
client.connect("mqtt3.thingspeak.com", 1883,60)
client.loop_forever()