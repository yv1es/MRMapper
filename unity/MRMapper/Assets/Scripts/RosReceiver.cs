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
            (data) => {
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
}




