using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Voronoi2;

public enum Mode
{
    Create,
    Select
}

public static class Maths2D
{

    public static float PseudoDistanceFromPointToLine(Vector2 a, Vector2 b, Vector2 c)
    {
        return Mathf.Abs((c.x - a.x) * (-b.y + a.y) + (c.y - a.y) * (b.x - a.x));
    }

    public static int SideOfLine(Vector2 a, Vector2 b, Vector2 c)
    {
        return (int)Mathf.Sign((c.x - a.x) * (-b.y + a.y) + (c.y - a.y) * (b.x - a.x));
    }

    public static int SideOfLine(float ax, float ay, float bx, float by, float cx, float cy)
    {
        return (int)Mathf.Sign((cx - ax) * (-by + ay) + (cy - ay) * (bx - ax));
    }

    public static bool PointInTriangle(Vector2 a, Vector2 b, Vector2 c, Vector2 p)
    {
        float area = 0.5f * (-b.y * c.x + a.y * (-b.x + c.x) + a.x * (b.y - c.y) + b.x * c.y);
        float s = 1 / (2 * area) * (a.y * c.x - a.x * c.y + (c.y - a.y) * p.x + (a.x - c.x) * p.y);
        float t = 1 / (2 * area) * (a.x * b.y - a.y * b.x + (a.y - b.y) * p.x + (b.x - a.x) * p.y);
        return s >= 0 && t >= 0 && (s + t) <= 1;

    }

    public static bool LineSegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float denominator = ((b.x - a.x) * (d.y - c.y)) - ((b.y - a.y) * (d.x - c.x));
        if (Mathf.Approximately(denominator, 0))
        {
            return false;
        }

        float numerator1 = ((a.y - c.y) * (d.x - c.x)) - ((a.x - c.x) * (d.y - c.y));
        float numerator2 = ((a.y - c.y) * (b.x - a.x)) - ((a.x - c.x) * (b.y - a.y));

        if (Mathf.Approximately(numerator1, 0) || Mathf.Approximately(numerator2, 0))
        {
            return false;
        }

        float r = numerator1 / denominator;
        float s = numerator2 / denominator;

        return (r > 0 && r < 1) && (s > 0 && s < 1);
    }

}

public class Vertex
{
    public Vector3 position;

    //The outgoing halfedge (a halfedge that starts at this vertex)
    //Doesnt matter which edge we connect to it
    public HalfEdge halfEdge;

    //Which triangle is this vertex a part of?
    public Triangle triangle;

    //The previous and next vertex this vertex is attached to
    public Vertex prevVertex;
    public Vertex nextVertex;

    //Properties this vertex may have
    //Reflex is concave
    public bool isReflex;
    public bool isConvex;
    public bool isEar;

    public Vertex(Vector3 position)
    {
        this.position = position;
    }

    //Get 2d pos of this vertex
    public Vector2 GetPos2D_XZ()
    {
        Vector2 pos_2d_xz = new Vector2(position.x, position.z);

        return pos_2d_xz;
    }
}

public class HalfEdge
{
    //The vertex the edge points to
    public Vertex v;

    //The face this edge is a part of
    public Triangle t;

    //The next edge
    public HalfEdge nextEdge;
    //The previous
    public HalfEdge prevEdge;
    //The edge going in the opposite direction
    public HalfEdge oppositeEdge;

    //This structure assumes we have a vertex class with a reference to a half edge going from that vertex
    //and a face (triangle) class with a reference to a half edge which is a part of this face 
    public HalfEdge(Vertex v)
    {
        this.v = v;
    }
}

public class Triangle
{
    //Corners
    public Vertex v1;
    public Vertex v2;
    public Vertex v3;

    //If we are using the half edge mesh structure, we just need one half edge
    public HalfEdge halfEdge;

    public Triangle(Vertex v1, Vertex v2, Vertex v3)
    {
        this.v1 = v1;
        this.v2 = v2;
        this.v3 = v3;
    }

    public Triangle(Vector3 v1, Vector3 v2, Vector3 v3)
    {
        this.v1 = new Vertex(v1);
        this.v2 = new Vertex(v2);
        this.v3 = new Vertex(v3);
    }

    public Triangle(HalfEdge halfEdge)
    {
        this.halfEdge = halfEdge;
    }

    //Change orientation of triangle from cw -> ccw or ccw -> cw
    public void ChangeOrientation()
    {
        Vertex temp = this.v1;

        this.v1 = this.v2;

        this.v2 = temp;
    }
}

public class PointManager : MonoBehaviour
{
    public Mode mode;

    private GameObject point;

    private Transform firstPathfinderPoint;
    private Transform secondPathfinderPoint;

    private new Camera camera;
    private LineRenderer hullLine;
    private Transform graphLines;
    private LineRenderer pathLine;

    private Transform[] hull;
    private Edge[] path;
    private Graph graph;
    private GrahamScan grahamScan;
    private Voronoi voronoi;
    private Pathfinder pathfinder;
    private List<Triangle> Tris = new List<Triangle>();

    private void Start()
    {
        mode = Mode.Create;

        point = Resources.Load<GameObject>("Prefabs/Point");

        camera = Camera.main;
        hullLine = GameObject.Find("Hull Line").GetComponent<LineRenderer>();
        graphLines = GameObject.Find("Graph Lines").transform;
        pathLine = GameObject.Find("Path Line").GetComponent<LineRenderer>();

        hull = new Transform[0];
        path = new Edge[0];
        graph = new Graph();
        grahamScan = new GrahamScan();
        voronoi = new Voronoi(0f);
        pathfinder = new Pathfinder();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (mode == Mode.Create)
            {
                mode = Mode.Select;
                camera.backgroundColor = Color.gray;
            }
            else
            {
                mode = Mode.Create;
                camera.backgroundColor = Color.white;
            }
        }

        switch (mode)
        {
            case Mode.Create:
                if (Input.GetMouseButtonDown(0))
                {
                    UpdatePoints();
                }
                break;
        }
    }

    public void SelectPoint(Transform point)
    {
        if (point == firstPathfinderPoint)
        {
            return;
        }

        if (path.Length > 0)
        {
            ResetAllPoints();
            UpdateHull();

            firstPathfinderPoint = null;
            secondPathfinderPoint = null;

            path = new Edge[0];
            pathLine.SetPositions(new Vector3[0]);
        }

        if (firstPathfinderPoint == null)
        {
            firstPathfinderPoint = point;
            firstPathfinderPoint.GetComponent<Image>().color = Color.blue;
        }
        else
        {
            secondPathfinderPoint = point;
            secondPathfinderPoint.GetComponent<Image>().color = Color.blue;
            path = pathfinder.CalculatePath(graph, graph.FindNode(firstPathfinderPoint), graph.FindNode(secondPathfinderPoint));

            pathLine.positionCount = path.Length + 1;

            pathLine.SetPosition(0, secondPathfinderPoint.position);

            //Desenha as linhas do segundo ponto selecionado até o primeiro seguindo os pontos já calculados
            for (int i = 0; i < path.Length - 1; i++)
            {
                float distanceNodeA = Mathf.Pow(firstPathfinderPoint.position.x - path[i].nodeA.transform.position.x, 2) + Mathf.Pow(firstPathfinderPoint.position.y - path[i].nodeA.transform.position.y, 2);
                float distanceNodeB = Mathf.Pow(firstPathfinderPoint.position.x - path[i].nodeB.transform.position.x, 2) + Mathf.Pow(firstPathfinderPoint.position.y - path[i].nodeB.transform.position.y, 2);

                if (distanceNodeA < distanceNodeB)
                {
                    pathLine.SetPosition(i + 1, path[i].nodeA.transform.position);
                }
                else
                {
                    pathLine.SetPosition(i + 1, path[i].nodeB.transform.position);
                }
            }
            pathLine.SetPosition(path.Length, firstPathfinderPoint.position);
        }
    }

    private void UpdatePoints()
    {
        Vector3 spawnPosition = camera.ScreenToWorldPoint(Input.mousePosition);
        spawnPosition.z = 0;

        if (CheckPointAvailability(spawnPosition))
        {
            GameObject newPoint = Instantiate(point, spawnPosition, Quaternion.identity, transform);
            newPoint.name = "Point";
            newPoint.GetComponent<Point>().manager = this;
            graph.AddNode(new Node(newPoint.transform));

            List<Transform> points = GetChildren();
            List<Vector3> myPoints2 = new List<Vector3>();
            // Calculate Hull
            if (points.Count > 2)
            {
                hull = grahamScan.CalculateHull(points);
                UpdateHull();
            }

            // Calculate Voronoi
            List<double> xPositions = new List<double>();
            List<double> yPositions = new List<double>();

            for (int i = 0; i < points.Count; i++)
            {
                Vector3 myPoints = new Vector3(points[i].transform.position.x, 0, points[i].transform.position.y);
                myPoints2.Add(myPoints);
                xPositions.Add(points[i].position.x);
                yPositions.Add(points[i].position.y);
            }
            List<GraphEdge> edges = new List<GraphEdge>();
            if (points.Count > 5)
            {
                Tris = TriangulatePolygon(myPoints2);
                //Debug.Log(Tris[0].v1.position.x);
                //foreach (Triangle tri in Tris)
                //{

                //    graph.AddEdge(new Edge(tri.v1.position,
                //                           tri.v2.position));


                //    graph.AddEdge(new Edge(tri.v1.position,
                //                           tri.v3.position));
                //}

              //  foreach (Triangle tri in Tris)
              //  {
              //      //Debug.Log(tri.v1.position.x);

              //      GameObject newLine = new GameObject();
              //      newLine.transform.parent = graphLines;
              //      LineRenderer line = newLine.AddComponent<LineRenderer>();

              //      line.material = hullLine.material;
              //      line.startWidth = 0.05f;
              //      line.startColor = Color.black;
              //      line.endColor = Color.black;

              //      line.positionCount = 2;
              //      line.SetPositions(new Vector3[]
              //      {
              //new Vector3(tri.v2.position.x, tri.v2.position.z, 0),
              //   new Vector3(tri.v3.position.x, tri.v3.position.z, 0)
              //      });
              //  }
              //  foreach (Triangle tri in Tris)
              //  {
              //      //Debug.Log(tri.v1.position.x);

              //      GameObject newLine = new GameObject();
              //      newLine.transform.parent = graphLines;
              //      LineRenderer line = newLine.AddComponent<LineRenderer>();

              //      line.material = hullLine.material;
              //      line.startWidth = 0.05f;
              //      line.startColor = Color.black;
              //      line.endColor = Color.black;

              //      line.positionCount = 2;
              //      line.SetPositions(new Vector3[]
              //      {
              // new Vector3(tri.v2.position.x, tri.v2.position.z, 0),
              //   new Vector3(tri.v3.position.x, tri.v3.position.z, 0)
              //      });
              //  }
            }
            if (points.Count > 2)
            {
                hull = grahamScan.CalculateHull(points);
                hull = points.ToArray();
                UpdateHull();
            }

            graph.ClearEdges();

           

            UpdateEdges();
        }
    }

    private void UpdateHull()
    {
        hullLine.positionCount = hull.Length;
        for (int i = 0; i < hull.Length; i++)
        {
            hull[i].GetComponent<RectTransform>().sizeDelta = new Vector2(12, 12);
            hull[i].GetComponent<Image>().color = Color.red;

            hullLine.SetPosition(i, hull[i].position);
        }
    }

    private void UpdateEdges()
    {
        for (int i = graphLines.childCount - 1; i >= 0; i--)
        {
            Destroy(graphLines.GetChild(i).gameObject);
        }

        Edge[] edges = graph.GetEdges();


        if (Tris != null)
        {
            foreach (Triangle tri in Tris)
            {
                //if (BelongsToHull(edge.nodeA.transform) && BelongsToHull(edge.nodeB.transform))
                //{
                //    continue;
                //}

                GameObject newLine = new GameObject();
                newLine.transform.parent = graphLines;
                LineRenderer line = newLine.AddComponent<LineRenderer>();

                line.material = hullLine.material;
                line.startWidth = 0.005f;
                line.startColor = Color.black;
                line.endColor = Color.black;

                line.positionCount = 2;
                line.SetPositions(new Vector3[]
                {
               new Vector3(tri.v2.position.x, tri.v2.position.z, 0),
                 new Vector3(tri.v3.position.x, tri.v3.position.z, 0)
                });
            }
            foreach (Triangle tri in Tris)
            {
                //if (BelongsToHull(edge.nodeA.transform) && BelongsToHull(edge.nodeB.transform))
                //{
                //    continue;
                //}

                GameObject newLine = new GameObject();
                newLine.transform.parent = graphLines;
                LineRenderer line = newLine.AddComponent<LineRenderer>();

                line.material = hullLine.material;
                line.startWidth = 0.005f;
                line.startColor = Color.black;
                line.endColor = Color.black;

                line.positionCount = 2;
                line.SetPositions(new Vector3[]
                {
               new Vector3(tri.v2.position.x, tri.v2.position.z, 0),
                 new Vector3(tri.v3.position.x, tri.v3.position.z, 0)
                });
            }
        }
    }

    private bool CheckPointAvailability(Vector3 point)
    {
        for (int i = 0; i < transform.childCount; i++)
        {
            if (point == transform.GetChild(i).position)
            {
                return false;
            }
        }

        return true;
    }

    private bool BelongsToHull(Transform point)
    {
        if (hull == null)
        {
            return false;
        }

        for (int i = 0; i < hull.Length; i++)
        {
            if (hull[i] == point)
            {
                return true;
            }
        }

        return false;
    }

    private void ResetAllPoints()
    {
        for (int i = 0; i < transform.childCount; i++)
        {
            ResetPoint(transform.GetChild(i));
        }
    }

    private void ResetPoint(Transform point)
    {
        point.GetComponent<RectTransform>().sizeDelta = new Vector2(8, 8);
        point.GetComponent<Image>().color = Color.black;
    }

    private List<Transform> GetChildren()
    {
        List<Transform> children = new List<Transform>();

        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);

            ResetPoint(child);
            children.Add(child);
        }

        return children;
    }
    public static List<Triangle> TriangulatePolygon(List<Vector3> points)
    {
        //The list with triangles the method returns
        List<Triangle> triangles = new List<Triangle>();

        //If we just have three points, then we dont have to do all calculations
        if (points.Count == 3)
        {
            triangles.Add(new Triangle(points[0], points[1], points[2]));

            return triangles;
        }



        //Step 1. Store the vertices in a list and we also need to know the next and prev vertex
        List<Vertex> vertices = new List<Vertex>();

        for (int i = 0; i < points.Count; i++)
        {
            vertices.Add(new Vertex(points[i]));
        }

        //Find the next and previous vertex
        for (int i = 0; i < vertices.Count; i++)
        {
            int nextPos = i + 1;

            int prevPos = i - 1;

            if (i == 0)
                prevPos = vertices.Count - 1;
            if (i == vertices.Count - 1)
                nextPos = 0;

            vertices[i].prevVertex = vertices[prevPos];

            vertices[i].nextVertex = vertices[nextPos];
        }


        bool convex = true;
        //Step 2. Find the reflex (concave) and convex vertices, and ear vertices
        for (int i = 0; i < vertices.Count; i++)
        {
            CheckIfReflexOrConvex(vertices[i]);
            if(vertices[i].isConvex == false)
            {
                convex = false;
            }
        }

        if (convex)
        {
            Debug.Log("polígono convexo");
        }
        else
        {
            Debug.Log("polígono concavo");
        }

        //Have to find the ears after we have found if the vertex is reflex or convex
        List<Vertex> earVertices = new List<Vertex>();

        for (int i = 0; i < vertices.Count; i++)
        {
            IsVertexEar(vertices[i], vertices, earVertices);
        }


       
        //Step 3. Triangulate!
        while (true && earVertices != null && convex == false)
        {
            //This means we have just one triangle left
            if (vertices.Count == 3)
            {
                //The final triangle
                triangles.Add(new Triangle(vertices[0], vertices[0].prevVertex, vertices[0].nextVertex));

                break;
            }

            //Make a triangle of the first ear
            Vertex earVertex = earVertices[0];

            Vertex earVertexPrev = earVertex.prevVertex;
            Vertex earVertexNext = earVertex.nextVertex;

            Triangle newTriangle = new Triangle(earVertex, earVertexPrev, earVertexNext);

            triangles.Add(newTriangle);

            //Remove the vertex from the lists
            earVertices.Remove(earVertex);

            vertices.Remove(earVertex);
                //Update the previous vertex and next vertex
                earVertexPrev.nextVertex = earVertexNext;
                earVertexNext.prevVertex = earVertexPrev;

                //...see if we have found a new ear by investigating the two vertices that was part of the ear

                CheckIfReflexOrConvex(earVertexPrev);
                CheckIfReflexOrConvex(earVertexNext);

                //Debug.Log(earVertexPrev.isConvex);
                //Debug.Log(earVertexNext.isConvex);

                earVertices.Remove(earVertexPrev);
                earVertices.Remove(earVertexNext);

                IsVertexEar(earVertexPrev, vertices, earVertices);
                IsVertexEar(earVertexNext, vertices, earVertices);
            
        }

        //Debug.Log(triangles.Count);

        if (convex)
        {
            float medianX = 0;
            float medianY = 0;
            for (int i = 0; i < vertices.Count; i++)
            {
                medianX += vertices[i].position.x;
                medianY += vertices[i].position.z;
            }
            medianX = medianX / vertices.Count;
            medianY = medianY / vertices.Count;
            Vector3 posMedian = new Vector3(medianX, 0, medianY);
            Vertex v = new Vertex(posMedian);
            //Debug.Log(medianX);
            triangles.Clear();
            Triangle newTriangle1 = new Triangle(vertices[vertices.Count - 1], v, vertices[0]);

            triangles.Add(newTriangle1);
            for (int i = 0; i < vertices.Count - 1; i++)
            {
                Triangle newTriangle = new Triangle(vertices[i], v, vertices[i + 1]);

                triangles.Add(newTriangle);
            }
        }
        return triangles;
    }



    //Check if a vertex if reflex or convex, and add to appropriate list
    //Check if a vertex if reflex or convex, and add to appropriate list
    private static void CheckIfReflexOrConvex(Vertex v)
    {
        v.isReflex = false;
        v.isConvex = false;

        //This is a reflex vertex if its triangle is oriented clockwise
        Vector2 a = v.prevVertex.GetPos2D_XZ();
        Vector2 b = v.GetPos2D_XZ();
        Vector2 c = v.nextVertex.GetPos2D_XZ();
        //Debug.Log(Maths2D.SideOfLine(a, b, c) == -1);
        if (!IsTriangleOrientedClockwise(a, b, c))
        {
            
            v.isReflex = true;
        }
        else
        {
            v.isConvex = true;
        }
    }



    //Check if a vertex is an ear
    private static void IsVertexEar(Vertex v, List<Vertex> vertices, List<Vertex> earVertices)
    {
        //A reflex vertex cant be an ear!
        if (v.isReflex)
        {
            return;
        }

        //This triangle to check point in triangle
        Vector2 a = v.prevVertex.GetPos2D_XZ();
        Vector2 b = v.GetPos2D_XZ();
        Vector2 c = v.nextVertex.GetPos2D_XZ();

        bool hasPointInside = false;

        for (int i = 0; i < vertices.Count; i++)
        {
            //We only need to check if a reflex vertex is inside of the triangle
            if (vertices[i].isReflex)
            {
                Vector2 p = vertices[i].GetPos2D_XZ();

                //This means inside and not on the hull
                if (IsPointInTriangle(a, b, c, p))
                {
                    hasPointInside = true;

                    break;
                }
            }
        }

        if (!hasPointInside)
        {
            earVertices.Add(v);
        }
    }
    public static float IsAPointLeftOfVectorOrOnTheLine(Vector2 a, Vector2 b, Vector2 p)
    {
        float determinant = (a.x - p.x) * (b.y - p.y) - (a.y - p.y) * (b.x - p.x);

        return determinant;
    }
    public static int ClampListIndex(int index, int listSize)
    {
        index = ((index % listSize) + listSize) % listSize;

        return index;
    }
    public static bool IsTriangleOrientedClockwise(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        bool isClockWise = true;

        float determinant = p1.x * p2.y + p3.x * p1.y + p2.x * p3.y - p1.x * p3.y - p3.x * p2.y - p2.x * p1.y;

        if (determinant > 0f)
        {
            isClockWise = false;
        }

        return isClockWise;
    }
    public static bool IsPointInTriangle(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p)
    {
        bool isWithinTriangle = false;

        //Based on Barycentric coordinates
        float denominator = ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));

        float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / denominator;
        float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / denominator;
        float c = 1 - a - b;

        //The point is within the triangle or on the border if 0 <= a <= 1 and 0 <= b <= 1 and 0 <= c <= 1
        //if (a >= 0f && a <= 1f && b >= 0f && b <= 1f && c >= 0f && c <= 1f)
        //{
        //    isWithinTriangle = true;
        //}

        //The point is within the triangle
        if (a > 0f && a < 1f && b > 0f && b < 1f && c > 0f && c < 1f)
        {
            isWithinTriangle = true;
        }

        return isWithinTriangle;
    }
    public static bool AreLinesIntersecting(Vector2 l1_p1, Vector2 l1_p2, Vector2 l2_p1, Vector2 l2_p2, bool shouldIncludeEndPoints)
    {
        bool isIntersecting = false;

        float denominator = (l2_p2.y - l2_p1.y) * (l1_p2.x - l1_p1.x) - (l2_p2.x - l2_p1.x) * (l1_p2.y - l1_p1.y);

        //Make sure the denominator is > 0, if not the lines are parallel
        if (denominator != 0f)
        {
            float u_a = ((l2_p2.x - l2_p1.x) * (l1_p1.y - l2_p1.y) - (l2_p2.y - l2_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;
            float u_b = ((l1_p2.x - l1_p1.x) * (l1_p1.y - l2_p1.y) - (l1_p2.y - l1_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;

            //Are the line segments intersecting if the end points are the same
            if (shouldIncludeEndPoints)
            {
                //Is intersecting if u_a and u_b are between 0 and 1 or exactly 0 or 1
                if (u_a >= 0f && u_a <= 1f && u_b >= 0f && u_b <= 1f)
                {
                    isIntersecting = true;
                }
            }
            else
            {
                //Is intersecting if u_a and u_b are between 0 and 1
                if (u_a > 0f && u_a < 1f && u_b > 0f && u_b < 1f)
                {
                    isIntersecting = true;
                }
            }

        }

        return isIntersecting;
    }

}