using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;


public class Receiver
{
    private readonly Thread receiverThread;
    private bool running;

    private int port;
    private string ip;
    private string tag; 

    public Receiver(string ip, int port, string tag)
    {
        this.port = port;   
        this.ip = ip;   
        this.tag = tag; 
        
        receiverThread = new Thread(ReceiverThread);
        receiverThread.IsBackground = true;
    }

    private void ReceiverThread(object callback)
    {
        while (running)
        {
            using (var client = new TcpClient(ip, port))
            {
                NetworkStream stream = client.GetStream();

                byte[] headerBuffer = new byte[4];
                while (running)
                {

                    // Read the header from the stream
                    int bytesRead = stream.Read(headerBuffer, 0, 4);
                    if (bytesRead < 4) break;

                    // Extract the length of the following data from the header
                    int dataLength = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(headerBuffer, 0));

                    // Read the data from the stream
                    byte[] dataBuffer = new byte[dataLength];
                    int totalBytesRead = 0;
                    while (totalBytesRead < dataLength)
                    {
                        bytesRead = stream.Read(dataBuffer, totalBytesRead, dataLength - totalBytesRead);
                        if (bytesRead == 0) break;
                        totalBytesRead += bytesRead;
                    }

                    if (totalBytesRead == dataLength)
                    {
                        ((Action<byte[]>)callback)(dataBuffer);
                    }

                }
            }
            Thread.Sleep(1000);
            log("Attempting to connect to MRMapper");
        }
    }

    private void log(string message) 
    { 
        Debug.Log("[" + tag + "] " + message);    
    }

    public void Start(Action<byte[]> callback)
    {
        running = true;
        receiverThread.Start(callback);
    }

    public void Stop()
    {
        running = false;
        receiverThread.Abort();
    }
}