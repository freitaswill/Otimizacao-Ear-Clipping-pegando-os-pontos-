using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Edge
{
    public Node nodeA;
    public Node nodeB;

    public Edge(Node nodeA, Node nodeB)
    {
        this.nodeA = nodeA;
        this.nodeB = nodeB;
    }
}
