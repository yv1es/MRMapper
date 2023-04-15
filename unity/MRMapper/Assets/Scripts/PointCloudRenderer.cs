using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

public class PointCloudRenderer : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;



    public void Start()
    {

        //ForceDotNet.Force();  // If you have multiple sockets in the following threads
        receiver = new Receiver();

        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {
                Debug.Log("Got bytes");
            });
        };

        receiver.Start(callback);
        Debug.Log("Receiver online");   
    }


    // Update is called once per frame
    public void Update()
    {
        if (!runOnMainThread.IsEmpty)
        {
            Action action;
            while (runOnMainThread.TryDequeue(out action))
            {
                action.Invoke();
            }
        }
    }


    private void OnDestroy()
    {
        receiver.Stop();
    }
}



// Define a class to hold the JSON data
[Serializable]
public class Data
{
    public int size;
    public string points; 
}



public class Receiver
{
    private readonly Thread receiveThread;
    private bool running;


    private int port = 5001;
    private String ip = "127.0.0.1";    

    public Receiver()
    {
        receiveThread = new Thread((object callback) =>
        {
            using (var client = new TcpClient(ip, port))
            {
                NetworkStream stream = client.GetStream();


                Debug.Log("Connected");

                byte[] headerBuffer = new byte[4];
                while (running) {

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
                        // Do something with the bytes data
                        Debug.Log("Received " + dataLength + " bytes");
                        ((Action<byte[]>)callback)(dataBuffer);
                    }

                }
                Debug.LogWarning("Exited receiver loop");
            }
        });
    }

    public void Start(Action<byte[]> callback)
    {
        running = true;
        receiveThread.IsBackground = true;
        receiveThread.Start(callback);
    }

    public void Stop()
    {
        running = false;
        receiveThread.Abort();
    }
}