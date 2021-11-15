using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pathfinder
{
    private Node begin, end;
    private List<Node> nodesOnHold, examinedNodes;
    private Node[] nodes;
    private Edge[] edges;

    public Edge[] CalculatePath(Graph graph, Node begin, Node end)
    {
        nodes = graph.GetNodes();
        edges = graph.GetEdges();

        ResetNodes();

        this.begin = begin;
        this.end = end;

        nodesOnHold = new List<Node>();
        examinedNodes = new List<Node>();

        nodesOnHold.Add(begin);
        examinedNodes.Add(begin);

        return Examine(begin);
    }

    //Examina os nodos e analisa qual tem a menor heuristica
    public Edge[] Examine(Node currentNode)
    {
        if (currentNode == end)
        {
            return GetPath();
        }

        nodesOnHold.Remove(currentNode);

        Node[] adjacencies = CheckAdjacencies(currentNode);

        foreach (Node adjacency in adjacencies)
        {
            if (examinedNodes.Find(item => adjacency == item) == null)
            {
                nodesOnHold.Add(adjacency);
                examinedNodes.Add(adjacency);

                adjacency.parent = currentNode;
                adjacency.distance = Mathf.Pow(adjacency.transform.position.x - end.transform.position.x, 2) +
                                     Mathf.Pow(adjacency.transform.position.y - end.transform.position.y, 2);
                adjacency.cost = currentNode.cost + 1;
            }
        }
        return Examine(CalculateNextNode());
    }

    //passa um nodo para funçao, a função vai analisar todas as edges, e as edges que conterem esse nodo vão retornar o próximo nodo
    public Node[] CheckAdjacencies(Node node)
    {
        List<Node> adjacencies = new List<Node>();

        foreach (Edge edge in edges)
        {
            if (node == edge.nodeA)
            {
                adjacencies.Add(edge.nodeB);
            }
            else if (node == edge.nodeB)
            {
                adjacencies.Add(edge.nodeA);
            }
        }

        return adjacencies.ToArray();
    }


    //Cuida de analisa e retornar sempre o nodo de menor heristica
    public Node CalculateNextNode()
    {
        float smallestHeuristic = Mathf.Infinity;
        Node smallestHeuristicNode = begin;

        foreach (Node node in nodesOnHold)
        {
            float heuristic = node.value;

            if (heuristic < smallestHeuristic)
            {
                smallestHeuristic = heuristic;
                smallestHeuristicNode = node;
            }
        }

        return smallestHeuristicNode;
    }

    private Edge[] GetPath()
    {
        Node current = end;
        Node parent = end.parent;

        List<Edge> path = new List<Edge>();

        while (current != begin)
        {
            path.Add(GetEdge(current, parent));

            current = parent;
            parent = current.parent;
        }

        return path.ToArray();

    }

    //Retorna a edge entre dois nodos
    private Edge GetEdge(Node nodeA, Node nodeB)
    {
        foreach (Edge edge in edges)
        {
            if ((nodeA == edge.nodeA && nodeB == edge.nodeB) || (nodeA == edge.nodeB && nodeB == edge.nodeA))
            {
                return edge;
            }
        }

        return null;
    }

    private void ResetNodes()
    {
        foreach (Node node in nodes)
        {
            node.cost = 1;
        }
    }
}
