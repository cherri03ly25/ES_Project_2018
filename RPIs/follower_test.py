from car_control import car_controller
from wifi_connect import wifi_connector
import sys

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
	
    car2 = car_controller(2)
    server = wifi_connector(2,'')

    while 1:
        msg = server.receive_from_client()
        cmd = int(msg)
	    
        if cmd == STOP:
            print('Client sent: STOP')
            car2.run_stop()
            server.send_to_client('Server replied: STOP')
        elif cmd == START:
            print('Client sent: START')
            car2.run_start()
            server.send_to_client('Server replied: START')	        
        elif cmd == BACK:
            print('Client sent: BACK')
            car2.run_back()
            server.send_to_client('Server replied: BACK')
        else:
            print('Client sent: ', msg)

if __name__ == "__main__":
    sys.exit(main())

