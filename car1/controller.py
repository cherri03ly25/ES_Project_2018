import socket #for wifi

from smbus2 import SMBusWrapper, i2c_msg #for i2c
import time

################################-I2C-##################################
# Master address (RPi)
DEVICE_ADDR = 0x15
DEVICE_BUS = 1
# Slave address (Arduino)
address = 0x4a
class controller:
    def write_to_arduino(running, s, m):
        with SMBusWrapper(DEVICE_BUS) as bus:
            bus.write_i2c_block_data(address, running, [s, m])
        return -1

    def read_from_arduino(self):
        with SMBusWrapper(DEVICE_BUS) as bus:
            bus.write_byte(address,2)
            msg=i2c_msg.read(address, 3)
            bus.i2c_rdwr(msg)
            data = list(msg)
        return data[0], data[1], data[2]

    def __init__(self):
        ################################-WIFI-##################################
        HOST = '10.100.25.103' # server IP or Hostname (found at terminal: $ hostname -I)
        PORT = 22000 # Pick an open Port (1000+ recommended), must match the server port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((HOST,PORT))

    def send_to_server(self, r,s,m):
        cmd = str(r) + ' ' + str(s) + ' ' + str(m)
        self.client.send(str.encode(cmd))
        print('sending msg to server')
        return -1;

    def receive_from_server(self):
        msg = self.client.recv(1024)
        return msg.decode('utf-8');


    #################-MAIN-########################
    # while True:
    #     r,s,m = read_from_arduino()
    #     print('Arduino sends:')
    #     print('s: ', s)
    #     print('m: ', m)
    #     send_to_server(r,s,m)
    #     msg = receive_from_server()
    #     print(msg)
    #     print('#########################')
    #     if m == 1:
    #         break
    # client.close()
