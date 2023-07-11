using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class AlignCameras : MonoBehaviour
{
    GameObject realSense;
    GameObject mainCamera;

    void Start()
    {
        realSense = GameObject.Find("RealSense");
        mainCamera = GameObject.Find("Main Camera");
    }

    void Update()
    {

        // find difference between 
        float scale = 1f; 
        Vector3 posDiff = mainCamera.transform.position - realSense.transform.position;
        posDiff = scale * posDiff;
        Quaternion rotDiff = Quaternion.Inverse(realSense.transform.rotation) * mainCamera.transform.rotation;
        rotDiff = Quaternion.Slerp(Quaternion.identity, rotDiff, scale);

        float tPos = 0.1f;
        float rPos = 0.1f;
        if (posDiff.magnitude > tPos || rotDiff.eulerAngles.magnitude > rPos)
        {
            transform.position = transform.position + posDiff;
            transform.rotation = transform.rotation * rotDiff;
            Debug.Log("Updating origin " + posDiff);
            Debug.Log("Updating origin " + rotDiff);
        }

  
    }
}
