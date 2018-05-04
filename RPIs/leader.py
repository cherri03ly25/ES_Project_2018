import socket #for wifi
from smbus2 import SMBusWrapper, i2c_msg #for i2c
import time

################################-I2C-##################################
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

def run_stop():
    with SMBusWrapper(DEVICE_BUS) as bus:
		bus.write_byte(address, STOP)
    print('STOP!')

def run_start():
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address, START)
    print('START!')

def run_faster():
    with SMBusWrapper(DEVICE_BUS) as bus:
		bus.write_byte(address, FASTER)
	print('FASTER!')

def run_slower():
    with SMBusWrapper(DEVICE_BUS) as bus:
    	bus.write_byte(address, SLOWER)
    print('SLOWER!')

def turn_right():
	with SMBusWrapper(DEVICE_BUS) as bus:
		bus.write_byte(address, RIGHT)
    print('RIGHT!')
    
def turn_left():
	with SMBusWrapper(DEVICE_BUS) as bus:
		bus.write_byte(address, LEFT)
    print('LEFT!')
    
def move_back():
	with SMBusWrapper(DEVICE_BUS) as bus:
		bus.write_byte(address, BACK)
    print('BACK!')
            
def write_to_arduino(run, steering, speed):
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_i2c_block_data(address, WRITE, [run, steering, speed])
    print('WRITE!')
    return -1
 
def read_from_arduino(): ##running r, steering s and speed m
    with SMBusWrapper(DEVICE_BUS) as bus:
        bus.write_byte(address, READ)
        msg=i2c_msg.read(address, 3)
        bus.i2c_rdwr(msg)
        data = list(msg)
    print('READ!')
    return data[0], data[1], data[2]

################################-WIFI-##################################
HOST = '192.168.1.102' # server IP or Hostname (found at terminal: $ hostname -I)
PORT = 22000 # Pick an open Port (1000+ recommended), must match the server port
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST,PORT))

def send_cmd_to_server(cmd):
    client.send(str.encode(str(cmd)))
    print('sending cmd to server')
    return -1

def send_data_to_server(r,s,m):
    cmd = str(r) + ' ' + str(s) + ' ' + str(m)
    client.send(str.encode(cmd))
    print('sending data to server')
    return -1

def receive_from_server():
    msg = client.recv(1024)  
    return msg.decode('utf-8')


#################-MAIN-########################
import termios, fcntl, sys, os
fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


try:
    while 1:
        try:
            k = sys.stdin.read(4) 
            if k == '\x1b[6~': # PgDn key
                #send_cmd_to_server(cmd)
                run_stop()
            elif k == '\x1b[5~': # PgUp key
                #send_cmd_to_server(cmd)
                run_start()
            elif k == '\x1b[A': # up arrow key
                run_faster()
            elif k == '\x1b[B': # down arrow key
                run_slower()
            elif k == '\x1b[C': # right arrow key
                turn_right()
            elif k == '\x1b[D': # left arrow key
                turn_left()
            elif k == 'b':
                move_back()
            else:
                print('Invalid command!')
                
        except IOError: 
            pass

finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    client.close()


