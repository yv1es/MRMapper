using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ReceiverObjects : MonoBehaviour
{
    private readonly ConcurrentQueue<Action> runOnMainThread = new ConcurrentQueue<Action>();
    private Receiver receiver;

    private List<GameObject> objects = new List<GameObject>();

    //private List<GameObject> gameObjects; 

    public void Start()
    {
        // ROS to unity coordinate correction 
        this.transform.rotation = Quaternion.Euler(90, 90, 180);

        // callback for receiver
        Action<byte[]> callback = (data) => {
            runOnMainThread.Enqueue(
            () => {


            });
        };

        // start receiver 
        receiver = new Receiver("127.0.0.1", 5004, "Object Receiver");
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


    private void RenderPlanarPatches(List<Transform> new_objects, List<int> object_labels)
    {
        // remove rendered planes
        while (objects.Count > 0)
        {
            GameObject g = objects[0];
            objects.Remove(g);
            Destroy(g);
        }

        for (int i = 0; i < new_objects.Count;i++)
        {
            int label = object_labels[i];   

        }

    }



    private void OnDestroy()
    {
        receiver.Stop();
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



