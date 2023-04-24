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
    private Receiver receiver;
   

    public void Start()
    {
        // callback for receiver 
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
                () => {
                
                // coordinate conversions from ROS to unity 
                Vector3 pos = new Vector3();
                pos.z = BitConverter.ToSingle(data, 0);
                pos.x = -BitConverter.ToSingle(data, 4);
                pos.y = BitConverter.ToSingle(data, 8);

                Quaternion rot = new Quaternion();
                rot.z = -BitConverter.ToSingle(data, 12);
                rot.x = BitConverter.ToSingle(data, 16);
                rot.y = -BitConverter.ToSingle(data, 20);
                rot.w = BitConverter.ToSingle(data, 24);
                
                // update camera transform 
                if (pos != Vector3.zero)
                {
                    transform.position = pos;  
                    transform.rotation = rot;  
                }
            });
        };

        // start receiver 
        receiver = new Receiver("127.0.0.1", 5002, "Odom Receiver");
        receiver.Start(callback);
    }


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


