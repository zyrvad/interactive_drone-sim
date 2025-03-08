using System.Collections.Generic;
using UnityEngine;

public static class SplineUtility
{
    public static List<Vector3> GenerateCatmullRomSpline(Vector3[] points, int resolution)
    {
        List<Vector3> smoothedPoints = new();

        for (int i = 0; i < points.Length - 1; i++)
        {
            Vector3 p0 = points[(i - 1 + points.Length - 1) % (points.Length - 1)];
            Vector3 p1 = points[i];
            Vector3 p2 = points[(i + 1) % (points.Length - 1)];
            Vector3 p3 = points[(i + 2) % (points.Length - 1)];

            for (int j = 0; j <= resolution; j++)
            {
                float t = j / (float)resolution;
                smoothedPoints.Add(InterpolateCatmullRom(p0, p1, p2, p3, t));
            }
        }

        smoothedPoints.Add(smoothedPoints[0]);
        return smoothedPoints;
    }

    private static Vector3 InterpolateCatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return 0.5f * (
            2 * p1 +
            (-p0 + p2) * t +
            (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
            (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t
        );
    }
}
