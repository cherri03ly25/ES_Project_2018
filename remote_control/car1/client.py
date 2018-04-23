import socket #for wifi
from smbus2 import SMBusWrapper, i2c_msg #for i2c
import time

################################-I2C-##################################
# Master address (RPi)
DEVICE_ADDR = 0x15
DEVICE_BUS = 1
# Slave address (Arduino)
address = 0x4a

def write_to_arduino(running, s, m):
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_i2c_block_data(address, running, [s, m])
    return -1

def read_from_arduino():
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address,2)
        msg=i2c_msg.read(address, 3)
        bus.i2c_rdwr(msg)
        data = list(msg)
    return data[0], data[1], data[2]

def read_rps_speed():
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address, 3)
        msg=i2c_msg.read(address, 2)
        bus.i2c_rdwr(msg)
        data = list(msg)
    return data[0], data[1]

################################-WIFI-##################################
HOST = '192.168.1.104' # server IP or Hostname (found at terminal: $ hostname -I)
PORT = 22000 # Pick an open Port (1000+ recommended), must match the server port
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST,PORT))

def send_to_server(r,s,m):
    cmd = str(r) + ' ' + str(s) + ' ' + str(m)
    client.send(str.encode(cmd))
    print('sending msg to server')
    return -1;

def receive_from_server():
    msg = client.recv(1024)  
    return msg.decode('utf-8');


#################-MAIN-########################
while True:
##    r,s,m = read_from_arduino()
##    print('Arduino sends:')
##    print('s: ', s)
##    print('m: ', m)
##    send_to_server(r,s,m)
##    msg = receive_from_server()
##    print(msg)
##    print('#########################')
##    if m == 1:
##        break
    #send commands to arduino
    var = input("Enter r s m: ")
    var = var.split()
    r = int(var[0])
    s = int(var[1])
    m = int(var[2])
    write_to_arduino(r,s,m)
    count = 0;
    while count <= 5:
        count += 1
        if r = 1:
            time.sleep(2)
            rps, speed = read_rps_speed()
            print('rps: ', rps,', speed :', speed)

     write_to_arduino(0, 90, 0)

##    send_to_server(r,s,m)
##    msg = receive_from_server()
    
client.close()


