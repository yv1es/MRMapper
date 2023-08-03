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

    Methods:
        __init__(self, ip, port, tag): Initializes the UnitySender instance.
        start(self): Starts the sender and listenes to a incoming Unity connection.
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
        self.MAX_SENDER_QUEUE_LENGTH = 5

    def start(self):
        self.socket = s.socket(s.AF_INET, s.SOCK_STREAM)
        self.socket.bind((self.ip, self.port)) 
        self.socket.listen()

        self.listener = threading.Thread(target=self.listenerThread)
        self.listener.deamon = True 
        self.listener.start()

        
    
    def listenerThread(self):
        while self.running:
            self.log("Listening for incoming unity connection")
            
            conn, _ = self.socket.accept()
            self.log("Established connection to unity")

            try:
                self.senderThread(conn)
            finally:
                conn.close()
            self.log("Connection to unity closed")


    def log(self, message):
        rospy.loginfo("[{}] {}".format(self.tag, message))


    def senderThread(self, conn):
        while self.running:
                try:
                    data = self.sender_queue.get(timeout=1)
                    conn.sendall(data)
                    self.sender_queue.task_done()
                except queue.Empty:
                    pass
                except Exception as e:
                    print("An exception occurred:", e)
                    return 

    # synchronized this function
    lock = threading.Lock()
    def send(self, data):
        with self.lock: 
            header = struct.pack('!I', len(data))
            message = header + data
            self.sender_queue.put(message)

            # Check if the queue has reached the maximum length
            if self.sender_queue.qsize() > self.MAX_SENDER_QUEUE_LENGTH:
                self.sender_queue.get()

    def stop(self):
        self.running = False
        self.socket.close()