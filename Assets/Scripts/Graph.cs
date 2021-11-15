using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Voronoi2;

public class Graph
{
    private List<Node> nodes;
    private List<Edge> edges;

    public Graph()
    {
        nodes = new List<Node>();
        edges = new List<Edge>();
    }

    public void AddNode(Node point)
    {
        nodes.Add(point);
    }

    public void AddEdge(Edge edge)
    {
        edges.Add(edge);
    }

    public Node[] GetNodes()
    {
        return nodes.ToArray();
    }

    public Edge[] GetEdges()
    {
        return edges.ToArray();
    }

    public Node FindNode(Transform transform)
    {
        foreach (Node node in nodes)
        {
            if (node.transform == transform)
            {
                return node;
            }
        }

        return null;
    }

    public void ClearEdges()
    {
        edges.Clear();
    }
}
