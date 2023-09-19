from time import sleep
from urllib import request
import random


def post_http(data):
    url = "https://api.thingspeak.com/update?api_key=27G90F1UEBXTY6IY&"
    url += data
    request.urlopen(url)
    print("http send ok ")



    # temp = random.randint(20,30)
    # humi = random.randint(80, 100)
    # rd = random.randint(0,100)

temp = 25
humi = 75
rd = 46

post = "field1="
post += str(temp)
post += "&field2="
post += str(humi)
post += "&field3="
post += str(rd)
    
post_http(post)


