using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class GrahamScan
{
    public Transform[] CalculateHull(List<Transform> points)
    {
        Transform lowestY = GetLowestY(points);

        points = points.OrderBy(point => Vector3.Angle(Vector3.right, point.position - lowestY.position)).ToList();

        for (int i = 2; i < points.Count - 1;)
        {
            float ccw = CheckConvex(points[i - 1].position, points[i].position, points[i + 1].position);

            if (ccw >= 0)
            {
                i++;
            }
            else
            {
                points.RemoveAt(i);
                i--;
            }
        }

        return points.ToArray();
    }

    private float CheckConvex(Vector3 pointA, Vector3 pointB, Vector3 pointC)
    {
        return (pointB.x - pointA.x) * (pointC.y - pointA.y) -
               (pointB.y - pointA.y) * (pointC.x - pointA.x);
    }

    private Transform GetLowestY(List<Transform> points)
    {
        Transform lowestY = points[0];

        for(int i = 1; i < points.Count; i++)
        {
            if (points[i].position.y <= lowestY.position.y)
            {
                // In case of same Y, gets the higher X
                if (points[i].position.y == lowestY.position.y &&
                    points[i].position.x < lowestY.position.x)
                {
                    continue;
                }

                lowestY = points[i];
            }
        }

        return lowestY;
    }
}
