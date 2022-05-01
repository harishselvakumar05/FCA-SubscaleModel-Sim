from ast import parse
import serial
import re
import _thread
from dataclasses import dataclass

@dataclass
class Packet:
    ch1: float
    ch2: float
    ch3: float
    ch4: float

ser = serial.Serial()
ser.port = 'COM4'
ser.baudrate = 57600
ser.open()
packet = Packet(0.0, 0.0, 0.0, 0.0)

def write():
    

try:
    _thread.start_new_thread(write,())          
except:
    print("Thread Exception")
while 1:
    pass   

while(ser.is_open):
    raw =  ser.readline().decode()
    id = ' '.join(re.findall(r"(\d+)\:", raw))
    parsed_line = ' '.join(re.findall(r"\[(\d+)\]", raw))
    if (id != ' ' and parsed_line != ' '):
        if(id == 1):
            packet.ch1 = parsed_line
        elif(id == 2):
            packet.ch2 = parsed_line
        elif(id == 3):
            packet.ch3 = parsed_line
        else:
            packet.ch4 = parsed_line

     
