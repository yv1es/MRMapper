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

public class ReceiveOdom : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private OdomReceiver receiver;
   

    public void Start()
    {
        receiver = new OdomReceiver();

        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {
                // do stuff here 

                Vector3 pos = new Vector3();
                pos.z = BitConverter.ToSingle(data, 0);
                pos.x = -BitConverter.ToSingle(data, 4);
                pos.y = BitConverter.ToSingle(data, 8);

                Quaternion rot = new Quaternion();
                rot.z = BitConverter.ToSingle(data, 12);
                rot.x = BitConverter.ToSingle(data, 16);
                rot.y = BitConverter.ToSingle(data, 20);
                rot.w = BitConverter.ToSingle(data, 24);

                this.transform.position = pos;  
                //rot = RosToUnityQuat * rot;  
                this.transform.rotation = rot;  
            });
        };

        receiver.Start(callback);
        Debug.Log("Receiver online");
    }


    // Create a rotation quaternion to convert from ROS to Unity
    private Quaternion RosToUnityQuat
    {
        get { return Quaternion.Euler(90, 0, 0); }// Quaternion.AngleAxis(Mathf.Rad2Deg * Mathf.PI / 2, new Vector3(1, 0, 0)); }
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




public class OdomReceiver
{
    private readonly Thread receiveThread;
    private bool running;


    private int port = 5002;
    private string ip = "127.0.0.1";

    public OdomReceiver()
    {
        receiveThread = new Thread((object callback) =>
        {
            using (var client = new TcpClient(ip, port))
            {
                NetworkStream stream = client.GetStream();


                Debug.Log("Odom Connected");

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
                        // Do something with the bytes data
                        Debug.Log("Odom Received " + dataLength + " bytes");
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