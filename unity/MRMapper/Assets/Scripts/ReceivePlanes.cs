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




public class ReceivePlanes : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;


    public void Start()
    {
        // ROS to unity coordinate correction 
        this.transform.rotation = Quaternion.Euler(90, 90, 180);

        // callback for receiver
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {

                Debug.Log("Got message"); 

            });
        };

        // start receiver 
        receiver = new Receiver("127.0.0.1", 5003, "Plane Receiver");
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
}


