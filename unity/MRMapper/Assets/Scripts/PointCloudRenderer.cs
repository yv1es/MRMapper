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


public class PointCloudRenderer : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;
    public float pointSize = 1f;

    Mesh mesh;
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;


    public void Start()
    {
        // ROS to unity coordinate correction 
        this.transform.rotation = Quaternion.Euler(90, 90, 180); 
        
        // setup point cloud mesh renderer 
        meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Particles/Standard Unlit"));
        meshRenderer.material.SetColor("_Color", Color.white);
        meshRenderer.material.SetFloat("_PointSize", pointSize);


        // callback for receiver
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {
                RenderPointCloud(data); 
            });
        };

        // start receiver 
        receiver = new Receiver("127.0.0.1", 5001, "Pcl Receiver");
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

    
    void RenderPointCloud(byte[] pointCloudData)
    {
        List<Vector3> points = new List<Vector3>();
        List<Color> colors = new List<Color>();
        int numPoints = pointCloudData.Length / 32;

        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * 32;
            float x = BitConverter.ToSingle(pointCloudData, offset);
            float y = BitConverter.ToSingle(pointCloudData, offset + 4);
            float z = -BitConverter.ToSingle(pointCloudData, offset + 8);
            points.Add(new Vector3(x, y, z));

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


