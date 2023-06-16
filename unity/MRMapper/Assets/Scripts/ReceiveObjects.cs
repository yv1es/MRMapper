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

    Mesh chairMesh; 

    public void Start()
    {
        GameObject meshObject = Resources.Load<GameObject>("56");
        chairMesh = meshObject.transform.GetChild(0).GetComponent<MeshFilter>().sharedMesh;
           


        if (!(chairMesh != null && chairMesh.vertexCount > 0 && chairMesh.triangles.Length > 0))
        {
            Debug.LogWarning("Invalid chairMesh. Please make sure it contains valid vertex and triangle data.");
        }

        Setup(port, log_tag, ProcessReceivedBytes);
    }

    private void ProcessReceivedBytes(byte[] data)
    {
        int numObjts = data.Length / 32;
        
        
        for (int i = 0; i < numObjts; i++) { 
            int offset = i * 32;

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
            Quaternion rot = RtabQuatToUnity(q) * Quaternion.Euler(-90, 0, 90);
               
            int class_id = (int)BitConverter.ToSingle(data, offset + 28);
            Debug.Log("Received a " +  class_id);


            GameObject go = new GameObject("chair");
            
            Transform transform = go.transform;
            transform.position = pos;
            transform.rotation = rot;

            go.AddComponent<MeshFilter>();
            go.AddComponent<MeshRenderer>();

            MeshFilter meshFilter = go.GetComponent<MeshFilter>();
            MeshRenderer meshRenderer = go.GetComponent<MeshRenderer>();
            meshFilter.mesh = chairMesh;

            Material defaultMaterial = new Material(Shader.Find("Standard"));
            meshRenderer.material = defaultMaterial;

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


