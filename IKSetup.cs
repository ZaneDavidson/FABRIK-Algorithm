using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class IKSetup : MonoBehaviour
{
    float totalLength;
    float[] lengthArr;
    Transform[] joints;
    Vector3[] jointPositions;
    Quaternion targetStartRot;
    Quaternion[] lengthRotationArray;
    Vector3[] lengthVectors;
    Vector3[] lastJointPositions;
    public int length;
    public Transform target;
    public Transform pole;
    public float delta = 0.001f;
    [Range(1,100)]
    public int iterations;
    public bool debugLines = false;
    public bool isGrounded = true;

    void Awake()
    {
        Init();
    }

    void Init()
    {
        //Initialize all variables and arrays from start positions and rotations
        lengthArr = new float[length];
        joints = new Transform[length + 1];
        jointPositions = new Vector3[length + 1];
        lengthRotationArray = new Quaternion[length + 1];
        lengthVectors = new Vector3[length + 1];
        lastJointPositions = new Vector3[length + 1];
        targetStartRot = target.rotation;

        var current = this.transform;

        //set initialized values
        for(int i = joints.Length - 1; i >= 0; i--)
        {
            joints[i] = current;
            lengthRotationArray[i] = joints[i].rotation;

            if(i == joints.Length - 1)
            {
                lengthVectors[i] = target.position - current.position;
            }
            else
            {
                lengthVectors[i] = joints[i + 1].position - current.position;
                lengthArr[i] = (joints[i + 1].position - current.position).magnitude;
                totalLength += lengthArr[i];
            }
            current = current.parent;
        }
    }

    void LateUpdate()
    {
        if(target == null)
        {
            Debug.Log("target null");
            return;
        }

        if(joints.Length != length + 1)
        {
            Debug.Log("reset ran");
            Init();
        }

        //getting positions
        for(int i = 0; i < joints.Length; i++)
        {   
            jointPositions[i] = joints[i].position;
            lastJointPositions[i] = jointPositions[i];
        }

        //change positions based on pole position
        if(pole != null)
        {
            for(int i = 1; i < jointPositions.Length - 1; i++)
            {
                var plane = new Plane(jointPositions[i + 1] - jointPositions[i - 1], jointPositions[i - 1]);
                var projectedPole = plane.ClosestPointOnPlane(pole.position);
                var projectedBone = plane.ClosestPointOnPlane(jointPositions[i]);
                var angle = Vector3.SignedAngle(projectedBone - jointPositions[i - 1], projectedPole - jointPositions[i - 1], plane.normal);
                jointPositions[i] = Quaternion.AngleAxis(angle, plane.normal) * (jointPositions[i] - jointPositions[i - 1]) + jointPositions[i - 1];
            }
        }

        
        if((this.gameObject.name == "foot.R_end" || this.gameObject.name == "foot.L_end") && isGrounded)
        {   
            Debug.Log("Foot");
        }
        else
        {
            IKSolve();
        }

        //finally, set new positions
        for(int i = 0; i < jointPositions.Length; i++)
        {
            if(i == jointPositions.Length - 1)
            {
                joints[i].rotation = target.rotation * Quaternion.Inverse(targetStartRot) * lengthRotationArray[i];
            }
            else
            {
                Quaternion match = Quaternion.FromToRotation(lengthVectors[i], (jointPositions[i + 1] - jointPositions[i]));
                joints[i].rotation = match * lengthRotationArray[i];
            }
            joints[i].position = jointPositions[i];
        }

    }

    void IKSolve()
    {
        //if target is farther away than the arm can reach far away
        if((target.position - joints[0].position).sqrMagnitude >= totalLength * totalLength)
        {
            //skip root, i = 0
            for(int i = 1; i < jointPositions.Length; i++)
            {
                jointPositions[i] = jointPositions[i - 1] + (target.position - joints[0].position).normalized * lengthArr[i - 1];
            }
        }
        else
        {
            //FABRIK implementation
            for(int i = 0; i < iterations && (target.position - joints[0].position).sqrMagnitude >= delta * delta; i++)
            {
                backwardIK();
                forwardIK();
            } 
        }

        //make sure arm cannot collide with body colliders
        for(int i = jointPositions.Length - 1; i > 0; i--)
        {
            RaycastHit hit;
            if(Physics.Raycast(jointPositions[i], (jointPositions[i - 1] - jointPositions[i]).normalized, out hit, (jointPositions[i - 1] - jointPositions[i]).magnitude))
            {
                if(hit.transform.gameObject.layer == 9)
                {
                    lastJointPositions.CopyTo(jointPositions, 0);
                }
            }
        }
    }

    //go back allong chain from leaf bone, set positions except for root
    private void backwardIK()
    {
        for(int i = jointPositions.Length - 1; i > 0; i--)
        {
            if(i == jointPositions.Length - 1)
            {
                jointPositions[i] = target.position;
            }
            else
            {
                jointPositions[i] = jointPositions[i + 1] + (jointPositions[i] - jointPositions[i + 1]).normalized * lengthArr[i];
            }
        }
    }

    //go forwards along chain from one after the root bone, set positions 
    private void forwardIK()
    {
        for(int i = 1; i < jointPositions.Length; i++)
        {
            jointPositions[i] = jointPositions[i - 1] + (jointPositions[i] - jointPositions[i - 1]).normalized * lengthArr[i - 1];

        }
    }
    
    void OnDrawGizmos()
    {
        if(debugLines == true)
        {
            //test code to show links between bones
            var current = this.transform;
            for (int i = 0; i < length && current != null && current.parent != null; i++)
            {
                var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
                Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
                Handles.color = Color.red;
                Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
                current = current.parent;
            }
        }
    } 
}