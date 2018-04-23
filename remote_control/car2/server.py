import socket #for wifi
from smbus2 import SMBusWrapper, i2c_msg #for i2c
import time

################################-I2C-##################################
# Master address (RPi)
DEVICE_ADDR = 0x15
DEVICE_BUS = 1
# Slave address (Arduino)
address = 0x4a

def write_to_arduino(running,s, m):
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_i2c_block_data(address, running, [s, m])
    return -1

def read_from_arduino():
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address,2)
        msg=i2c_msg.read(address, 3)
        bus.i2c_rdwr(msg)
        data = list(msg)
    return data[0], data[1], data[3]

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
    data = msg.split()
    r = int(str.encode(data[0]))
    s = int(str.encode(data[1]))
    m = int(str.encode(data[2]))
    return r, s, m

def send_to_client(msg):
    conn.send(str.encode(msg))
    print('Replied')
    return -1;


#################-MAIN-########################
server = setupServer()
conn = setupConnection()
while True:
    r,s,m = receive_from_client()
##    print('Client has sent:')
##    print('r: ', r)
##    print('s: ', s)
##    print('m: ', m)
    
    write_to_arduino(r,s,m)
    print('writing msg to arduino')
    msg = 'server received: running=' + str(r) + ', s=' + str(s) + ', m=' + str(m)
    send_to_client(msg)
    print('########################')
    if m == 1:
        break
conn.close()



