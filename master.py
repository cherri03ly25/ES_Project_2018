#import smbus
import time
from smbus2 import SMBusWrapper, i2c_msg

# Master address for Raspi 3
DEVICE_ADDR = 0x15

DEVICE_BUS = 1

#bus = smbus.SMBus(DEVICE_BUS)

# Slave address for Arduino
address = 0x4a

def write_to_arduino(s, m):
    #bus.write_byte_data(address, 0x40, value)
    #bus.write_byte(address, value)
    with SMBusWrapper(1) as bus:
        bus.write_i2c_block_data(address, 0, [s, m])
    return -1


def read_from_arduino():
    #number = bus.read_byte_data(address, 0x40)
    ## read_byte_data will first send the cmd 0x40 to arduino
    ## so the value on arduino changes to 0x40
    ##number = bus.read_byte(address)

    with SMBusWrapper(1) as bus:
    # read a block of 2 bytes from address, offset 0
        bus.write_byte(address,1)
        msg=i2c_msg.read(address, 2)
        bus.i2c_rdwr(msg)
        data = list(msg)
        print(data)

        #data = bus.read_i2c_block_data(address, 1, 2)
##        data = []
##        data.append(bus.read_byte_data(address,1))
##        data.append(bus.read_byte_data(address,2))
   
    return data[0], data[1]


def send_to_otherRPi(s,m):
    #wifi_write_data(ID,s,m)
    return 0;
    
def receive_from_otherRPi():
    #wifi_read_data(ID)
    return 0;

while True:
    #send commands to arduino
    var = input("command s m: ")
    var = var.split()
    s = int(var[0])
    m = int(var[1])
    write_to_arduino(s,m)
    
    #request data from arduino
    s,m = read_from_arduino()
    print('s: ', s)
    print('m: ', m)
    time.sleep(0.5)
    




