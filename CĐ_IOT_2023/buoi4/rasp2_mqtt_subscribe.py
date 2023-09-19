import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print("Connected with results code {}".format(rc))
    channel_ID = "2272720"
    client.subscribe("channels/%s/subscribe" % channel_ID)

def on_disconnect(client, userdata, rc):
    print("disconnect from broker")
    
def on_message(client, userdata, message):
    print("debug") 
    print(message.payload.decode())
    data_fields = message.payload.decode()
    data_fields = json.loads(data_fields)
    # print(type(data_fields))
    # print(data_fields)
    # print(type(data_fields["field1"]))  # nhiet do
    # print(type(data_fields["field2"]))  # do am
    # print(type(data_fields["field3"]))  # rand
    rand = int(data_fields["field3"])
    humi = int(data_fields["field2"])
    temp = int(data_fields["field1"])

    print("lcd: \nNhiet do: %s - Do am: %s - Rand: %s" %(temp,humi,rand))
    
    if temp > 37:
        print("buzzer on")
    elif temp < 31:
        print("buzzer off")
    else:
        print("ko doi temp")
    
    if humi > 90:
        print("relay on")
    elif humi < 80:
        print("relay off")
    else:
        print("ko doi relay")

    if rand > 50:
        print("led on")
    elif rand < 50:
        print("led_off")
    else:
        print("vao rand")

    


client_id = 'CBkJDTIVCzsaNBA0BiYgCxM'
client = mqtt.Client(client_id)

# gan cac chuong trinh con
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.username_pw_set(username='CBkJDTIVCzsaNBA0BiYgCxM', password= 'uxiAQcEHk7LlSuIu7jm3cmxo')
client.connect("mqtt3.thingspeak.com", 1883,60)
client.loop_forever()

