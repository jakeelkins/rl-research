import sys
import socket
import fcntl, os
import errno
from time import sleep

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost',10101))
fcntl.fcntl(s, fcntl.F_SETFL, os.O_NONBLOCK)

while True:
    try:
        msg = s.recv(16383)
    except socket.error as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            sleep(1)
            print('No data available')
            continue
        else:
            # a "real" error occurred
            print(e)
            sys.exit(1)
    else:
        # got a message, do something :)
        data = msg.decode('utf-8')
        #print(type(data))
        print(data)

        print('sending ack...')
        #ACcmd1_str = "5.0 Point SC[0].B[0] Primary Vector [0.0 0.0 1.0] at MOON\nEOF"
        
        ACcmd1_str = "Ack\n"
        sleep(0.5)
        ACcmd1 = bytes(ACcmd1_str, 'utf-8')
        success = s.sendall(ACcmd1)    #note: sendall sends ALL data, b is bytes string
        
        if success == None:
            print('ack sent.')
