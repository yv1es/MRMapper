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
using UnityEditor;
using UnityEngine;
using UnityEngine.XR;




public class ReceivePlanes : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;

    private List<GameObject> planes = new List<GameObject>(); 

    //private List<GameObject> gameObjects; 

    public void Start()
    {
        // ROS to unity coordinate correction 
        this.transform.rotation = Quaternion.Euler(90, 90, 180);

        // callback for receiver
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {

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
        while (planes.Count > 0) {
            GameObject g = planes[0];
            planes.Remove(g);
            Destroy(g); 
        }


        // Sort the points based on their coordinates
        Array.Sort(edgePoints, new Vector3Comparer());

        // Create a new game object and add necessary components
        GameObject rectangleObject = new GameObject("Plane");
        rectangleObject.AddComponent<MeshFilter>();
        rectangleObject.AddComponent<MeshRenderer>();

        planes.Add(rectangleObject);

        // ROS to unity coordinate correction 
        rectangleObject.transform.rotation = Quaternion.Euler(90, 90, 180);

        // Set the rectangle material
        MeshRenderer meshRenderer = rectangleObject.GetComponent<MeshRenderer>();

        // Create the mesh for the rectangle
        Mesh rectangleMesh = new Mesh();

        // Assign the vertices to the mesh
        rectangleMesh.vertices = edgePoints;

        // Define the indices for the triangles
        int[] indices = new int[]
        {
            0, 2, 3, 1, 0, 3,
            2, 0, 3, 0, 1, 3,
        };

        // Assign the indices to the mesh
        rectangleMesh.triangles = indices;

        // Calculate the normals and bounds for the mesh
        rectangleMesh.RecalculateNormals();
        rectangleMesh.RecalculateBounds();

        // Assign the mesh to the mesh filter
        MeshFilter meshFilter = rectangleObject.GetComponent<MeshFilter>();
        meshFilter.sharedMesh = rectangleMesh;
    }






    private void OnDestroy()
    {
        receiver.Stop();
    }
}




public class Vector3Comparer : IComparer<Vector3>
{
    public int Compare(Vector3 a, Vector3 b)
    {
        // Compare x component
        if (a.x < b.x)
            return -1;
        else if (a.x > b.x)
            return 1;

        // Compare y component
        if (a.y < b.y)
            return -1;
        else if (a.y > b.y)
            return 1;

        // Compare z component
        if (a.z < b.z)
            return -1;
        else if (a.z > b.z)
            return 1;

        return 0; // Vectors are equal
    }
}


