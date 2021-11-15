using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    public Transform transform;
    public Node parent;
    public float cost;
    public float distance;

    public Node(Transform transform)
    {
        this.transform = transform;
        cost = 1;
        distance = 0;
    }

    public float value
    {
        get
        {
            return cost + distance;
        }
    }
}
