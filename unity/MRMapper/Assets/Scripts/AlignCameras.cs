using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class AlignCameras : MonoBehaviour
{
    GameObject realSense;
    GameObject oculus;
    GameObject offset; 

    // define the position of the Oculus relative to the RealSense
    public Vector3 realSenseToOculusPos = new Vector3(0, -0.05f, 0);
    public Quaternion realSenseToOculusRot = Quaternion.Euler(0, 0, 0); 


    void Start()
    {
        realSense = GameObject.Find("RealSense");
        oculus = GameObject.Find("Oculus");
        offset = GameObject.Find("Oculus Offset"); 
    }

    void Update()
    {
        // compute where the Oculus should be positioned 
        Vector3 shouldPos = realSenseToOculusPos + realSense.transform.position ;
        Quaternion shouldRot = realSenseToOculusRot * realSense.transform.rotation;

        // where the oculus is currently positioned
        Vector3 isPos = oculus.transform.position;
        Quaternion isRot = oculus.transform.rotation;   


        // compute the correction 
        Vector3 posDiff = shouldPos - isPos;
        Quaternion rotDiff = shouldRot * Quaternion.Inverse(isRot);

        float absPosDiff = posDiff.magnitude;
        float absRotDiff = 2f * Mathf.Rad2Deg * Mathf.Acos(Mathf.Abs(rotDiff.w));


        float tPos = 0.1f;
        float tRot = 10f;


        if (absPosDiff > tPos || absRotDiff > tRot)
        {
            offset.transform.position = posDiff + offset.transform.position;
            offset.transform.rotation = rotDiff * offset.transform.rotation;
            Debug.Log("Resetting Oculus offset"); 
        }

  
    }
}
