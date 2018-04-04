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
HOST = '' # should be left empty
PORT = 22000 # Pick an open Port (1000+ recommended), must match the client port

def setupServer():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print ('Socket created')
    try:
        server.bind((HOST, PORT))
    except socket.error as msg:
        print(msg)
    print('Socket bind complete')
    return server

def setupConnection():
    server.listen(1) #allows one connection at a time
    conn, address = server.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    return conn

def receive_from_client():
    msg = conn.recv(1024)
    msg = msg.decode('utf-8')
    msg = msg.split()
    s = str.encode(msg[0])
    m = str.encode(msg[1])
    return 90, 0

def send_to_client(msg):
    conn.send(str.encode(msg))
    print('Replied')
    return -1;


#################-MAIN-########################
server = setupServer()
conn = setupConnection()
while True:
    s,m = receive_from_client()
    print('Client has sent:')
    print('s: ', s)
    print('m: ', m)
    write_to_arduino(s,m)
    print('writing s, m to arduino')
    msg = 'server received: s=' + str(s) + ', m=' + str(m)
    send_to_client(msg)
    if m == 0:
        break
conn.close()

