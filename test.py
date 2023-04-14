
import socket as s 
import time 
import json     

HOST = s.gethostname() 
PORT = 5002


sock = None
connected = False

def setupSocket():
    sock = s.socket(s.AF_INET, s.SOCK_STREAM)
    sock.connect((HOST, PORT))  
    return sock


def sendToUnity(msg):
    
    json_data = json.dumps(msg)
    sock.sendall(json_data.encode())


def main():
    print("Connecting")
    global sock 
    while True:
        try:
            sock = setupSocket()
            break
        except:
            print("Connection attempt to unity")
            time.sleep(1)

    print("setup")

if __name__ == '__main__':
    main()