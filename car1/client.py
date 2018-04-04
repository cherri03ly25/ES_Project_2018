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
    return data[0], data[1]

################################-WIFI-##################################
HOST = '192.168.1.33' # server IP or Hostname (found at terminal: $ hostname -I)
PORT = 22000 # Pick an open Port (1000+ recommended), must match the server port
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST,PORT))

def send_to_server(s,m):
    cmd = str(s) + ' ' + str(m)
    client.send(str.encode(cmd))
    print('sending s, m to server')
    return -1;

def receive_from_server():
    msg = client.recv(1024)  
    return msg.decode('utf-8');


#################-MAIN-########################
while True:
    s,m = read_from_arduino()
    print('Arduino sends:')
    print('s: ', s)
    print('m: ', m)
    send_to_server(s,m)
    msg = receive_from_server()
    print(msg)
    if m == 0:
        break
client.close()
