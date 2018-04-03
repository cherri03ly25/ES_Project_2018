import socket

HOST = '192.168.43.120' # Server IP or Hostname
PORT = 12345 # Pick an open Port (1000+ recommended), must match the client sport
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'

#managing error exception
try:
        s.bind((HOST, PORT))
except socket.error:
        print 'Bind failed '

s.listen(5)
print 'Socket awaiting messages'
(conn, addr) = s.accept()
print 'Connected'

# awaiting for message
while True:
        data = conn.recv(1024)
        print 'I sent a message back in response to: ' + data
        reply = ''

        # process your message
        if data == 'Hello':
                reply = 'Hi, back!'
        elif data == 'This is important':
                reply = 'OK, I have done the important thing you have asked me!'

        #and so on and on until...
        elif data == 'quit':
                conn.send('Terminating')
                break
        else:
                reply = 'Unknown command'

        # Sending reply
        conn.send(reply)
conn.close() # Close connections
