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


public class ReceivePointCloud : RosReceiver
{

    Mesh mesh;
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;
    public float pointSize = 1f;


    int port = 5001;
    string log_tag = "Pcd Receiver";


    public void Start()
    {
        // setup point cloud mesh renderer 
        meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Particles/Standard Unlit"));
        meshRenderer.material.SetColor("_Color", Color.white);
        meshRenderer.material.SetFloat("_PointSize", pointSize);


        Setup(port, log_tag, ProcessReceivedBytes);
    }

    private void ProcessReceivedBytes(byte[] data)
    {
        RenderPointCloud(data);
    }


    void RenderPointCloud(byte[] pointCloudData)
    {
        List<Vector3> points = new List<Vector3>();
        List<Color> colors = new List<Color>();
        int numPoints = pointCloudData.Length / 32;

        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * 32;
            float[] v = new float[3];
            v[0] = BitConverter.ToSingle(pointCloudData, offset);
            v[1] = BitConverter.ToSingle(pointCloudData, offset + 4);
            v[2] = BitConverter.ToSingle(pointCloudData, offset + 8);
            points.Add(RtabVecToUnity(v));

            Color color = new Color(pointCloudData[offset + 18] / 255f, pointCloudData[offset + 17] / 255f, pointCloudData[offset + 16] / 255f);
            colors.Add(color);
        }

        // update mesh 
        mesh = new Mesh();
        mesh.SetVertices(points);
        mesh.SetColors(colors);
        mesh.SetIndices(Enumerable.Range(0, numPoints).ToArray(), MeshTopology.Points, 0);
        meshFilter.mesh = mesh;
    }
}


