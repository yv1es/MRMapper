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
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.XR;




public class RosReceiver : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;

    public void Setup(int port, string log_tag, Action<byte[]> action)
    {
        // callback for receiver
        Action<byte[]> callback =
            (data) =>
            {
                runOnMainThread.Enqueue(() => { action(data); });
            };

        // start receiver 
        receiver = new Receiver("127.0.0.1", port, log_tag);
        receiver.Start(callback);
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


    public Vector3 RtabVecToUnity(float[] pos)
    {
        Quaternion q = Quaternion.Euler(90, 90, 180);

        Vector3 vec = new Vector3();
        vec.x = pos[0];
        vec.y = pos[1];
        vec.z = -pos[2];
        return q * vec;
    }

    public Quaternion RtabQuatToUnity(float[] quat)
    {
        Quaternion q = new Quaternion();
        q.x = quat[1];
        q.y = -quat[2];
        q.z = -quat[0];
        q.w = quat[3];
        return q;
    }

    private class Receiver
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
}




