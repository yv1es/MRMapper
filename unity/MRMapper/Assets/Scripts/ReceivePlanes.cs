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




public class ReceivePlanes : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;


    public void Start()
    {
        // ROS to unity coordinate correction 
        this.transform.rotation = Quaternion.Euler(90, 90, 180);

        // callback for receiver
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {

                Debug.Log("Got message");

                List<Vector3[]> planes = new List<Vector3[]>();

                int numPlanes = data.Length / 48; // 4 bytes per value, 3 values per point, 4 points per plane
                for (int i = 0; i < numPlanes; i++)
                {
                    int offset = i * 48;
                    Vector3[] p = new Vector3[4];
                    for (int j = 0; j < 4; j++) {
                        int point_offset = offset + j * 12;
                        float x = BitConverter.ToSingle(data, point_offset);
                        float y = BitConverter.ToSingle(data, point_offset + 4);
                        float z = -BitConverter.ToSingle(data, point_offset + 8); // right to left handed coordinate system change
                        Vector3 v = new Vector3(x, y, z);
                        p[j] = v; 
                    }
                    planes.Add(p);  
                }

                Debug.Log(planes); 

                foreach (var plane in planes)
                {
                    RenderPlanarPatch(plane); 
                }
            });
        };

        // start receiver 
        receiver = new Receiver("127.0.0.1", 5003, "Plane Receiver");
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

    private void RenderPlanarPatch(Vector3[] edgePoints)
    {
        // Create a new game object to represent the planar patch
        GameObject planarPatchObject = new GameObject("PlanarPatch");

        // Create a mesh filter and renderer component for the game object
        MeshFilter meshFilter = planarPatchObject.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = planarPatchObject.AddComponent<MeshRenderer>();

        // Create a mesh with the edge points
        Mesh mesh = new Mesh();
        mesh.vertices = edgePoints;

        // Define the triangle indices
        int[] triangleIndices = new int[] { 0, 1, 2, 0, 2, 3 };
        mesh.triangles = triangleIndices;

        // Calculate vertex normals (assuming planar patch is not bent)
        Vector3 normal = Vector3.Cross(edgePoints[1] - edgePoints[0], edgePoints[2] - edgePoints[0]).normalized;
        Vector3[] normals = new Vector3[] { normal, normal, normal, normal };
        mesh.normals = normals;

        // Assign the mesh to the mesh filter
        meshFilter.mesh = mesh;

        // Add a default material to the mesh renderer
        meshRenderer.material = new Material(Shader.Find("Standard"));
    }


    private void OnDestroy()
    {
        receiver.Stop();
    }
}


