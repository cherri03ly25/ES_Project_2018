from car_control import car_controller
from wifi_connect import wifi_connector
import termios, fcntl, sys, os

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

def main():

    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
	
    car1 = car_controller(1)
    client = wifi_connector(1, '192.168.1.101')

    try:
	while 1:
	    try:
		k = sys.stdin.read(4) 
		if k == '\x1b[6~': # PgDn key
	            client.send_to_server(str(STOP))
	            car1.run_stop()
                    msg = client.receive_from_server()
                    print(msg)
		elif k == '\x1b[5~': # PgUp key
	            client.send_to_server(str(START))
	            car1.run_start()
                    msg = client.receive_from_server()
                    print(msg)
		elif k == '\x1b[A': # up arrow key
		    car1.run_faster()
	        elif k == '\x1b[B': # down arrow key
		    car1.run_slower()
		elif k == '\x1b[C': # right arrow key
	            car1.turn_right()
                elif k == '\x1b[D': # left arrow key
	            car1.turn_left()
                elif k == 'b':
		    client.send_to_server(str(BACK))
		    car1.run_back()
                    msg = client.receive_from_server()
                    print(msg)
		else:
		    print('Invalid command!')
		        
	    except IOError: 
		pass

    finally:
	termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

if __name__ == "__main__":
    sys.exit(main())


