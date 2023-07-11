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


public class ReceiveOdom : RosReceiver
{

    int port = 5002;
    string log_tag = "Odom Receiver";

    GameObject xrOrigin; 

    public void Start() {
        Setup(port, log_tag, ProcessReceivedBytes);
        xrOrigin = GameObject.Find("XR Origin");
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

        // update camera transform 
        //if (pos != Vector3.zero)
        //{
        //    transform.position = pos;
        //    transform.rotation = rot;
        //}

       

        Transform childTransform = this.transform; 
        Vector3 globalPosition = childTransform.TransformPoint(Vector3.zero);
        Quaternion globalRotation = childTransform.rotation;

        Vector3 oculusPos = globalPosition;
        Quaternion oculusRot = globalRotation;

        Vector3 diffPos = cameraPos - oculusPos;
        Quaternion diffRot = cameraRot * Quaternion.Inverse(oculusRot);

        // translation and rotation threshold 
        float tThreshold = 0.1f;
        float rThreshold = 3f; 

        if (Vec3Max(diffPos) > tThreshold || Vec3Max(diffRot.eulerAngles) > rThreshold + 5000)
        {
            //xrOrigin.transform.position = diffPos;
            //xrOrigin.transform.rotation = diffRot;
            //Debug.Log("Recentered Oculus"); 
        }

        Debug.Log("Position diff:" + diffPos);
        Debug.Log("Camera pos:" + cameraPos);
        //Debug.Log("Rotation diff:" + diffRot.eulerAngles);

    }

    private float Vec3Max(Vector3 v)
    {
        return Mathf.Max(MathF.Max(v.x, v.y), v.z); 
    }






}


