from time import sleep
from urllib import request
import random

def post_http(data):
    url = "https://api.thingspeak.com/update?api_key=27G90F1UEBXTY6IY&" + data
    request.urlopen(url)
    print("http send ok ")

while True:
    temp = random.randint(20,40)
    humi = random.randint(60, 100)
    rd = random.randint(0,100)

    post = "field1=%s&field2=%s&field3=%s" %(temp,humi,rd)
    # print(post)
    post_http(post)
    sleep(20)

