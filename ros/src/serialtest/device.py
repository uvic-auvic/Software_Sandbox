import pty
import os

if __name__ == '__main__':
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    print("Listening on Port '%s'\nWaiting for input..." % port)
    while True:
        data = os.read(master, 4)
        print("Serial Device recieved: '%s'"  % data)
        os.write(master, "ACK\n")