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
using Unity.XR.Oculus.Input;
using UnityEngine;
using UnityEngine.XR;
using static UnityEditor.PlayerSettings;

/*
 * This script moves the RealSense camera object. 
 */
public class ReceiveOdom : RosReceiver
{

    int port = 5002;
    string log_tag = "Odom Receiver";
    GameObject realSense;
    
    public void Start() {
        Setup(port, log_tag, ProcessReceivedBytes);
        realSense = GameObject.Find("RealSense");
    }   

    private void ProcessReceivedBytes(byte[] data)
    {
        float[] v = new float[3];
        v[0] = BitConverter.ToSingle(data, 0);
        v[1] = BitConverter.ToSingle(data, 4);
        v[2] = BitConverter.ToSingle(data, 8);
        Vector3 cameraPos = RtabVecToUnity(v);

        float[] q = new float[4];
        q[0] = BitConverter.ToSingle(data, 12);
        q[1] = BitConverter.ToSingle(data, 16);
        q[2] = BitConverter.ToSingle(data, 20);
        q[3] = BitConverter.ToSingle(data, 24);
        Quaternion cameraRot = RtabQuatToUnity(q);

        //update RealSense transform
        if (cameraPos != Vector3.zero)
        {
            realSense.transform.position = transform.position + cameraPos;
            realSense.transform.rotation = transform.rotation * cameraRot;
        }

    }


}


