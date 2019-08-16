import socket
import logging
import threading
import time

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )

global value

value=0

def socket_server():

	HOST = ''  # Standard loopback interface address (localhost)
	PORT = 6000         # Port to listen on (non-privileged ports are > 1023)

	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	    s.bind((HOST, PORT))
	    print('listening...')
	    s.listen()
	    conn, addr = s.accept()

	    gap = 10
	    value = 0 

	    with conn:
	        print('Connected by', addr)
	        while True:
	            data = conn.recv(1024)
	            if not data:
	                break
	            conn.sendall(data)

	            while(True):
	            	value = (value+1)%gap
	            	conn.send(value.to_bytes(2, byteorder='big'))
	            	command = conn.recv(1024)

	            	print(command)

	            	if (command == 's'):
	            		gap = gap + 10 % 30

            		if (command == 'q'):
            			s.close()
            			return
            			
def main():

	socket_server()


	

if __name__ == "__main__":
    main()