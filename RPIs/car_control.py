#i2c
from smbus2 import SMBusWrapper, i2c_msg
import time

# Master address (RPi)
DEVICE_ADDR = 0x15
DEVICE_BUS = 1
# Slave address (Arduino)
address = 0x4a

##speed values
MIN = 50
MAX = 100

##steering values
RIGHTMOST = 60
MIDDLE = 90
LEFTMOST = 120

##commands
STOP = 0
START = 1
FASTER = 2
SLOWER = 3
RIGHT = 4
LEFT = 5
BACK = 6
WRITE = 7
READ = 8

class car_controller:
    
    def __init__(self, id):
        self.__id = id;
        print('Car ', self.__id, ' is ready to run!')
	    
    def __del__(self):
        print ('Car ', self.__id, ' says goodbye!') 
		
	def run_stop(self):
    	with SMBusWrapper(DEVICE_BUS) as bus:
			bus.write_byte(address, STOP)
    	print('STOP!')

	def run_start(self):
    	with SMBusWrapper(DEVICE_BUS) as bus:
        	bus.write_byte(address, START)
    	print('START!')

	def run_faster(self):
    	with SMBusWrapper(DEVICE_BUS) as bus:
			bus.write_byte(address, FASTER)
		print('FASTER!')

	def run_slower(self):
    	with SMBusWrapper(DEVICE_BUS) as bus:
    		bus.write_byte(address, SLOWER)
    	print('SLOWER!')

	def turn_right(self):
		with SMBusWrapper(DEVICE_BUS) as bus:
			bus.write_byte(address, RIGHT)
		print('RIGHT!')
		
	def turn_left(self):
		with SMBusWrapper(DEVICE_BUS) as bus:
			bus.write_byte(address, LEFT)
		print('LEFT!')
		
	def run_back(self):
		with SMBusWrapper(DEVICE_BUS) as bus:
			bus.write_byte(address, BACK)
		print('BACK!')
            
	def write_to_arduino(self, run, steering, speed):
		with SMBusWrapper(DEVICE_BUS) as bus:
		    bus.write_i2c_block_data(address, WRITE, [run, steering, speed])
		print('WRITE!')
	 
	def read_from_arduino(self): ##run r, steering s and speed m
		with SMBusWrapper(DEVICE_BUS) as bus:
		    bus.write_byte(address, READ)
		    msg=i2c_msg.read(address, 3)
		    bus.i2c_rdwr(msg)
		    data = list(msg)
		print('READ!')
		return data[0], data[1], data[2]


