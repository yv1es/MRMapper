import rospy
import socket as s 
import threading
import struct
import threading
import queue


class UnitySender:
    def __init__(self, ip, port, tag) -> None:        
        self.ip = ip 
        self.port = port 
        self.tag = tag
        self.sender_queue = queue.Queue()
        self.running = True
        self.conn = None
        

    def start(self):
        self.socket = s.socket(s.AF_INET, s.SOCK_STREAM)
        self.socket.bind((self.ip, self.port)) 
        self.socket.listen()
        self.log("Listening incoming unity connection")
        self.conn, _ = self.socket.accept()
        self.log("Connected to unity")

        self.sender = threading.Thread(target=self.senderThread)
        self.sender.daemon = True
        self.sender.start()
        self.log("Sender thread running")
 

    def log(self, message):
        rospy.loginfo("[{}] {}".format(self.tag, message))


    def senderThread(self):
        while self.running:
                try:
                    data = self.sender_queue.get(timeout=1)
                    self.conn.sendall(data)
                    self.sender_queue.task_done()
                except queue.Empty:
                    pass


    def send(self, data):
        header = struct.pack('!I', len(data))
        message = header + data
        self.sender_queue.put(message)


    def stop(self):
        self.running = False
        self.conn.close()
        self.socket.close()