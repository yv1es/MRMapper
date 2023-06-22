#!/usr/bin/env python3
import rospy
import socket as s 
import threading
import struct
import threading
import queue


class UnitySender:
    """
    A class for sending data (bytes) to Unity over a socket connection.

    Attributes:
        ip (str): The IP address to connect to.
        port (int): The port number to connect to.
        tag (str): A tag to identify the sender mainly for logging.
        sender_queue (Queue): A queue to hold the data to be sent.
        running (bool): A flag indicating if the sender is running.
        conn (socket): The socket connection to Unity.

    Methods:
        __init__(self, ip, port, tag): Initializes the UnitySender instance.
        start(self): Starts the sender and establishes a connection to Unity.
        log(self, message): Logs a message with the specified tag.
        senderThread(self): The thread function for sending data to Unity.
        send(self, data): Adds data to the sender queue for sending.
        stop(self): Stops the sender and closes the connection to Unity.
    """

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