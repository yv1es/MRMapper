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
using static UnityEditor.PlayerSettings;


public class ReceiveObjects : RosReceiver
{

    int port = 5004;
    string log_tag = "Objt Receiver";


    public void Start()
    {
        Setup(port, log_tag, ProcessReceivedBytes);
    }

    private void ProcessReceivedBytes(byte[] data)
    {
        int numObjts = data.Length / 32;
        
        
        for (int i = 0; i < numObjts; i++) { 
            int offset = i * 28;

            float[] v = new float[3];
            v[0] = BitConverter.ToSingle(data, offset + 0);
            v[1] = BitConverter.ToSingle(data, offset + 4);
            v[2] = BitConverter.ToSingle(data, offset + 8);
            Vector3 pos = RtabVecToUnity(v);

            float[] q = new float[4];
            q[0] = BitConverter.ToSingle(data, offset + 12);
            q[1] = BitConverter.ToSingle(data, offset + 16);
            q[2] = BitConverter.ToSingle(data, offset + 20);
            q[3] = BitConverter.ToSingle(data, offset + 24);
            Quaternion rot = RtabQuatToUnity(q);
        }
        
        for (int j = 0; j < numObjts; j++) {
            int offset = numObjts * 28;
            int class_id = data[offset+j];
            string class_name = classes[class_id];
        }

    }



    string[] classes = {
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush" };
}


