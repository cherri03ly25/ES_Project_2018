import socket #for wifi
from smbus2 import SMBusWrapper, i2c_msg #for i2c
import time

################################-I2C-##################################
# Master address (RPi)
DEVICE_ADDR = 0x15
DEVICE_BUS = 1
# Slave address (Arduino)
address = 0x4a

def write_to_arduino(s, m):
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_i2c_block_data(address, 0, [s, m])
    return -1

def read_from_arduino():
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address,1)
        msg=i2c_msg.read(address, 2)
        bus.i2c_rdwr(msg)
        data = list(msg)
        print(data)
    return data[0], data[1]

################################-WIFI-##################################
HOST = '192.168.43.120' # server IP or Hostname
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

def send_to_server(s,m):
    cmd = str(s)+' '+str(m)
    s.send(cmd)
    return -1;

def receive_from_server():
    msg = s.recv(1024)  
    return msg;


#################-MAIN-########################
while True:
    s,m = read_from_arduino()
    print('s: ', s)
    print('m: ', m)
    send_to_server(s,m)
    msg = receive_from_server()
    print(msg)

