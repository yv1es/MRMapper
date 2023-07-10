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
using UnityEngine.AI;
using UnityEngine.XR;


public class ReceivePointCloud : RosReceiver
{

    int port = 5001;
    string log_tag = "Pcd Receiver";
    
    public float pointSize = 1f;
    GameObject pointCloud; 


    public void Start()
    {
        // setup point cloud game object 
        pointCloud = new GameObject("Point Cloud");
        pointCloud.transform.SetParent(transform);

        // setup mesh filter and renderer
        MeshFilter meshFilter = pointCloud.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = pointCloud.AddComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Particles/Standard Unlit"));
        meshRenderer.material.SetColor("_Color", Color.white);
        meshRenderer.material.SetFloat("_PointSize", pointSize);


        Setup(port, log_tag, ProcessReceivedBytes);
    }


    private void ProcessReceivedBytes(byte[] data)
    {
        // deserialize points and mesh 
        List<Vector3> points = new List<Vector3>();
        List<Color> colors = new List<Color>();
        int numPoints = data.Length / 32;
        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * 32;
            float[] v = new float[3];
            v[0] = BitConverter.ToSingle(data, offset);
            v[1] = BitConverter.ToSingle(data, offset + 4);
            v[2] = BitConverter.ToSingle(data, offset + 8);
            points.Add(RtabVecToUnity(v));

            Color color = new Color(data[offset + 18] / 255f, data[offset + 17] / 255f, data[offset + 16] / 255f);
            colors.Add(color);
        }


        // update mesh 
        Mesh mesh = pointCloud.GetComponent<MeshFilter>().mesh;
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32; // to allow for more points
        mesh.SetVertices(points);
        mesh.SetColors(colors);
        mesh.SetIndices(Enumerable.Range(0, numPoints).ToArray(), MeshTopology.Points, 0);
    }


}


