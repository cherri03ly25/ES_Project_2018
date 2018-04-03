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
PORT = 12345 # Pick an open Port (1000+ recommended), must match the client port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')

#managing error exception
try:
    s.bind((HOST, PORT))
except socket.error:
    print ('Bind failed')

s.listen(5)
print ('Socket awaiting messages')
(conn, addr) = s.accept()
print ('Connected')

def receive_from_client():
    msg = conn.recv(1024)
    msg = msg.split()
    s = int(msg[0])
    m = int(msg[1])
    ##conn.close()
    return s,m

def send_to_client(msg):
    conn.send(msg)
    ##conn.close()
    return -1;


#################-MAIN-########################
while True:
    s,m = receive_from_client()
    print('s: ', s)
    print('m: ', m)
    write_to_arduino(s,m)
    msg = 'received: s=' + str(s) +', m='+str(m)
    send_to_client(msg)

