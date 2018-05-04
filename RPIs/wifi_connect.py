import socket #for wifi
import time

## ID
CLIENT = 1
SERVER = 2

class wifi_connector:
    
    def __init__(self, id):
        self.__id = id
        if self.__id == CLIENT:           
            HOST = '192.168.1.102' # server IP or Hostname (found at terminal: $ hostname -I)
            PORT = 22000 # Pick an open Port (1000+ recommended), must match the server port
            self.__client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__client.connect((HOST,PORT))
		    print('Client is connected to: ', HOST)		    
	    else if self.__id == SERVER:
	        HOST = '' # should be left empty
            PORT = 22000 # Pick an open Port (1000+ recommended), must match the client port
            self.__server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.__server.bind((HOST, PORT))
            except socket.error as msg:
                print(msg)
            self.__server.listen(1) #allows one connection at a time
            self.__conn, address = server.accept()
            print('Server is connected to: ' + address[0] + ':' + str(address[1]))
		else:
            print('Invalid id: ', self.__id, ' => No connection')
	        
    def __del__(self):
        if self.__id == CLIENT:
            self.__client.close()
            print('Client closed connection!')
        else if self.__id == SERVER:
            self.__conn.close()
            ##self.__server.close()
            print('Server closed connection!')
    
    ## client ##
           
    def send_to_server(self, cmd): # cmd in string
        self.__client.send(str.encode(cmd))
        return -1;
        
    def receive_from_server(self):
        raw = self.__client.recv(1024)
		msg = raw.decode('utf-8')
        return msg; # return string
    
    ## server ##
    
    def receive_from_client(self):
        raw = self.__conn.recv(1024)
        msg = raw.decode('utf-8')
		cmd = str.encode(msg)
        ##data = msg.split()
        ##r = int(str.encode(data[1]))
        ##s = int(str.encode(data[2]))
        ##m = int(str.encode(data[3]))
        return cmd # return string
        
    def send_to_client(msg): # msg in string
        self.__conn.send(str.encode(msg))
        return -1;

