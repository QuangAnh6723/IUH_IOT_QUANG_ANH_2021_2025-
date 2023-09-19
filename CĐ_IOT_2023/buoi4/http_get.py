from urllib import request, parse
from time import sleep
import json

def thingspeak_http_get():
    api_key_read = "2X30FU2I919EJDHZ"
    channel_ID = "2272720"
    #https://api.thingspeak.com/channels/2272720/feeds.json?api_key=2X30FU2I919EJDHZ&results=2
    url = "https://api.thingspeak.com/channels/" + channel_ID + "/feeds.json?api_key=" + api_key_read+"&results=1"
    r = request.urlopen(url)

    respone_data = r.read().decode()
    respone_data = json.loads(respone_data)
    fields = respone_data['feeds']
 
    temp = fields[0]['field1']
    humi = fields[0]['field2']
    rand = fields[0]['field3']
    print(temp)
    print(humi)
    print(rand)

 
def main():
    # thingspeak_http_get()

    while(True):
        thingspeak_http_get()
        sleep(2)


main()