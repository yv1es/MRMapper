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
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.XR;




public class ReceivePlanes : RosReceiver
{
    int port = 5003;
    string log_tag = "Plane Receiver"; 

    // list to store the received planes 
    List<GameObject> planes = new List<GameObject>();


    public void Start()
    {
        Setup(port, log_tag, ProcessReceivedBytes);
    }

    private void ProcessReceivedBytes(byte[] data)
    {
        int numPlanes = data.Length / 52; // 4 bytes per value, 3 values per point, 4 points per plane gives 48 bytes, plus 4 bytes for the label
        for (int i = 0; i < numPlanes; i++)
        {
            // deserialize corner points and class_id 
            int offset = i * 52;
            Vector3[] corners = new Vector3[4];
            for (int j = 0; j < 4; j++)
            {
                int point_offset = offset + j * 12;
                float x = BitConverter.ToSingle(data, point_offset);
                float y = BitConverter.ToSingle(data, point_offset + 4);
                float z = BitConverter.ToSingle(data, point_offset + 8); 
                Vector3 v = RtabVecToUnity(new float[] { x, y, z });
                corners[j] = v;
            }
            int class_id = (int)BitConverter.ToSingle(data, offset + 48);

            GameObject p = CreatePlaneGameObject(corners, class_id);

            // update the plane list 
            if (i < planes.Count)
            {
                // update existing planes
                GameObject old_plane = planes[i];   
                planes[i] = p;
                Destroy(old_plane); 
            }
            else
            {
                planes.Add(p);  
            }
            
        }

        string s = "Printing vertices:\n\n"; 
        for (int i = 0; i < planes.Count; i++)
        {
            GameObject p = planes[i];
            Mesh mesh = p.GetComponent<MeshFilter>().sharedMesh;
            Vector3[] v = mesh.vertices;
            
            s += "\nVertices of " + p.name + ":\n";
            for (int j  = 0; j < v.Length; j++)
            {
                Vector3 c = v[j] + p.transform.position;
                s += "\t" + c.ToString() + "\n";
            }
        }
        Debug.Log(s);

    }


    private GameObject CreatePlaneGameObject(Vector3[] corners, int class_id)
    {
        string label = classes[class_id];

        // Sort the points based on their coordinates
        Array.Sort(corners, new Vector3Comparer());

        // Create a new game object and add necessary components
        GameObject plane = new GameObject(label + " plane");
        plane.transform.parent = transform;
        
        // Add mesh filter, mesh renderer and material 
        plane.AddComponent<MeshFilter>();
        plane.AddComponent<MeshRenderer>();
        MeshRenderer meshRenderer = plane.GetComponent<MeshRenderer>();
        MeshFilter meshFilter = plane.GetComponent<MeshFilter>();
        Material defaultMaterial = new Material(Shader.Find("VR/SpatialMapping/Wireframe"));
        meshRenderer.material = defaultMaterial;

        // the position of the game object should be in the middle of the mesh 
        Vector3 cornersMean = (corners[0] + corners[1] + corners[2] + corners[3]) / 4;
        corners[0] -= cornersMean; corners[1] -= cornersMean; corners[2] -= cornersMean; corners[3] -= cornersMean;
        plane.transform.position = cornersMean;

        // set the orientation to point in the normal of the plane
        //Vector3 normal = GetNormal(cornersMean, corners[0], corners[1]);
        //Quaternion rot = Quaternion.LookRotation(normal, corners[1] - corners[0]);
        //corners[0] =  Quaternion.Inverse(rot) * corners[0]; corners[1] = Quaternion.Inverse(rot) * corners[1]; corners[2] = Quaternion.Inverse(rot) * corners[2]; corners[3] = Quaternion.Inverse(rot) * corners[3];
        //plane.transform.rotation = rot; 

        // Create the mesh
        Mesh rectangleMesh = new Mesh();
        rectangleMesh.vertices = corners;
        rectangleMesh.triangles = new int[] { 0, 2, 3, 1, 0, 3, 2, 0, 3, 0, 1, 3 }; // 4 triangles that form a closed rectangle 
        rectangleMesh.RecalculateNormals();
        rectangleMesh.RecalculateBounds();
        meshFilter.sharedMesh = rectangleMesh;

        
        return plane; 
    }


    // Get the normal to a triangle from the three corner points, a, b and c.
    Vector3 GetNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        // Find vectors corresponding to two of the sides of the triangle.
        Vector3 side1 = b - a;
        Vector3 side2 = c - a;

        // Cross the vectors to get a perpendicular vector, then normalize it.
        return Vector3.Cross(side1, side2).normalized;
    }

    private class Vector3Comparer : IComparer<Vector3>
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





