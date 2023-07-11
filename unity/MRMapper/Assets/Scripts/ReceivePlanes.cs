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
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.XR;




public class ReceivePlanes : RosReceiver
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;

    // this list saves all planes that have been drawn, such that they can be removed later 
    private List<GameObject> rendered_planes = new List<GameObject>();


    int port = 5003;
    string log_tag = "Plane Receiver"; 


    public void Start()
    {
        Setup(port, log_tag, ProcessReceivedBytes);
    }

    private void ProcessReceivedBytes(byte[] data)
    {
        List<Vector3[]> planes_corners = new List<Vector3[]>();
        List<int> planes_labels = new List<int>();

        int numPlanes = data.Length / 52; // 4 bytes per value, 3 values per point, 4 points per plane gives 48 bytes, plus 4 bytes for the label
        for (int i = 0; i < numPlanes; i++)
        {
            int offset = i * 52;
            Vector3[] p = new Vector3[4];
            for (int j = 0; j < 4; j++)
            {
                int point_offset = offset + j * 12;
                float x = BitConverter.ToSingle(data, point_offset);
                float y = BitConverter.ToSingle(data, point_offset + 4);
                float z = BitConverter.ToSingle(data, point_offset + 8); 
                Vector3 v = RtabVecToUnity(new float[] { x, y, z });
                p[j] = v;
            }
            planes_corners.Add(p);

            int class_id = (int)BitConverter.ToSingle(data, offset + 48);
            planes_labels.Add(class_id);
        }
        RenderPlanarPatches(planes_corners, planes_labels);
    }


    private void RenderPlanarPatches(List<Vector3[]> planes_corners, List<int> planes_labels)
    {
        // remove previously rendered planes
        while (rendered_planes.Count > 0)
        {
            GameObject g = rendered_planes[0];
            rendered_planes.Remove(g);
            Destroy(g);
        }


        for (int i = 0; i < planes_corners.Count(); i++)
        {
            Vector3[] corners = planes_corners[i];
            int l = planes_labels[i];
            string label = classes[l];

            // Sort the points based on their coordinates
            Array.Sort(corners, new Vector3Comparer());

            // Create a new game object and add necessary components
            GameObject rectangleObject = new GameObject(label + "_plane");
            rectangleObject.AddComponent<MeshFilter>();
            rectangleObject.AddComponent<MeshRenderer>();

            

            rendered_planes.Add(rectangleObject);

            // ROS to unity coordinate correction 
            // rectangleObject.transform.rotation = Quaternion.Euler(90, 90, 180);

            // Set the rectangle material
            MeshRenderer meshRenderer = rectangleObject.GetComponent<MeshRenderer>();
            Material defaultMaterial = new Material(Shader.Find("Standard"));
            meshRenderer.material = defaultMaterial;

            // Create the mesh for the rectangle
            Mesh rectangleMesh = new Mesh();

            // Assign the vertices to the mesh
            rectangleMesh.vertices = corners;

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


