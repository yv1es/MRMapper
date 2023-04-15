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
    public float pointSize = 0.01f;

    Mesh mesh;
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;


    public void Start()
    {
        meshFilter = gameObject.AddComponent<MeshFilter>();
        
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Particles/Standard Unlit"));
        meshRenderer.material.SetColor("_Color", Color.white);
        meshRenderer.material.SetFloat("_PointSize", pointSize);


        //ForceDotNet.Force();  // If you have multiple sockets in the following threads
        receiver = new Receiver();

        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {
                RenderPointCloud(data); 
            });
        };

        receiver.Start(callback);
        Debug.Log("Receiver online");   
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
            float z = BitConverter.ToSingle(pointCloudData, offset + 8);
            points.Add(new Vector3(x, y, z));

            Color color = new Color(pointCloudData[offset + 18] / 255f, pointCloudData[offset + 17] / 255f, pointCloudData[offset + 16] / 255f);
            colors.Add(color);
        }

        
        mesh = new Mesh();
        mesh.SetVertices(points);
        mesh.SetColors(colors);
        mesh.SetIndices(Enumerable.Range(0, numPoints).ToArray(), MeshTopology.Points, 0);

        meshFilter.mesh = mesh;


    }
}



// Define a class to hold the JSON data
[Serializable]
public class Data
{
    public int size;
    public string points; 
}



public class Receiver
{
    private readonly Thread receiveThread;
    private bool running;


    private int port = 5001;
    private String ip = "127.0.0.1";    

    public Receiver()
    {
        receiveThread = new Thread((object callback) =>
        {
            using (var client = new TcpClient(ip, port))
            {
                NetworkStream stream = client.GetStream();


                Debug.Log("Connected");

                byte[] headerBuffer = new byte[4];
                while (running) {

                    // Read the header from the stream
                    int bytesRead = stream.Read(headerBuffer, 0, 4);
                    if (bytesRead < 4) break;

                    // Extract the length of the following data from the header
                    int dataLength = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(headerBuffer, 0));

                    // Read the data from the stream
                    byte[] dataBuffer = new byte[dataLength];
                    int totalBytesRead = 0;
                    while (totalBytesRead < dataLength)
                    {
                        bytesRead = stream.Read(dataBuffer, totalBytesRead, dataLength - totalBytesRead);
                        if (bytesRead == 0) break;
                        totalBytesRead += bytesRead;
                    }

                    if (totalBytesRead == dataLength)
                    {
                        // Do something with the bytes data
                        Debug.Log("Received " + dataLength + " bytes");
                        ((Action<byte[]>)callback)(dataBuffer);
                    }

                }
                Debug.LogWarning("Exited receiver loop");
            }
        });
    }

    public void Start(Action<byte[]> callback)
    {
        running = true;
        receiveThread.IsBackground = true;
        receiveThread.Start(callback);
    }

    public void Stop()
    {
        running = false;
        receiveThread.Abort();
    }
}