using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using System.Collections.Concurrent;
using System.Threading;
using UnityEditor.Experimental.GraphView;

public class PositionCamera : MonoBehaviour
{

    private const int Port = 5001;
    private IPAddress Ip = IPAddress.Parse("127.0.0.1");
    private Thread receiveThread;
    private TcpListener listener;
    private readonly ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    void Start()
    {
        // Start a new thread to receive JSON data
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        // Check if there are any new messages in the queue
        while (messageQueue.TryDequeue(out string message))
        {
            // Process the message in the main thread
            Debug.Log("Received message: " + message);
        }
    }

    private void ReceiveData()
    {
        
        Debug.Log("Connecting"); 
       
        var client = new TcpClient();
        client.Connect(Ip, Port);

        Debug.Log("Connection established");

        var stream = client.GetStream();
        var data = new byte[1024];
        
        var jsonData = "";
        while (true)
        {
            var bytes = stream.Read(data, 0, data.Length);
            jsonData = Encoding.ASCII.GetString(data, 0, bytes);

            messageQueue.Enqueue(jsonData); 

            // Deserialize JSON string to C# object
            //var obj = JsonConvert.DeserializeObject(jsonData);
            // Do something with obj here
        }

        // Clean up
        //stream.Close();
        //client.Close();

    }


    private void OnDestroy()
    {
        // Stop the receive thread and close the listener
        if (receiveThread != null)
        {
            receiveThread.Abort();
        }

        if (listener != null)
        {
            listener.Stop();
        }
    }
}