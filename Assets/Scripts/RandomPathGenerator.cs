using UnityEngine;
using System.Collections.Generic;

public class RandomPathGenerator : MonoBehaviour
{
    [SerializeField] private int pointsCount = 10;
    [SerializeField] private float radius = 10f;
    [SerializeField] private float yVariation = 5f;
    [SerializeField] private int splineResolution = 10;

    [SerializeField] private PathFollower follower;
    [SerializeField] private GameObject[] spawnObjects;
    [SerializeField] private Vector3 spawnOffset = new(0, 0, 30);

    [SerializeField] private bool enableY = false;
    [SerializeField] private bool useStraightPath = false;

    private Vector3[] pathPoints;
    private List<Vector3> smoothedPathPoints;

    public void Start()
    {
        GenerateAndAssignPaths();
    }

    public void GenerateAndAssignPaths()
    {
        pathPoints = useStraightPath ? GenerateStraightLinePath(pointsCount, transform.position) : GenerateClosedLoopPath(pointsCount, radius, transform.position);
        smoothedPathPoints = SplineUtility.GenerateCatmullRomSpline(pathPoints, splineResolution);

        foreach (GameObject obj in spawnObjects)
        {
            obj.transform.position = smoothedPathPoints[0] - spawnOffset;
        }

        follower.transform.localPosition = smoothedPathPoints[0];
        follower.StartFollowing(smoothedPathPoints);
    }

    Vector3[] GenerateClosedLoopPath(int count, float rad, Vector3 origin)
    {
        Vector3[] points = new Vector3[count + 1];
        float angleStep = 360f / count;

        for (int i = 0; i < count; i++)
        {
            float angle = i * angleStep * Mathf.Deg2Rad;
            float randomOffset = Random.Range(-rad * 0.3f, rad * 0.3f);
            float randomY = Random.Range(-yVariation, yVariation);

            if (enableY)
            {
                points[i] = new Vector3(
                    Mathf.Cos(angle) * (rad + randomOffset),
                    randomY,
                    Mathf.Sin(angle) * (rad + randomOffset)
                ) + origin;
            }
            else
            {
                points[i] = new Vector3(
                    Mathf.Cos(angle) * (rad + randomOffset),
                    0,
                    Mathf.Sin(angle) * (rad + randomOffset)
                ) + origin;
            }
        }

        points[count] = points[0];
        return points;
    }

    Vector3[] GenerateStraightLinePath(int count, Vector3 origin)
    {
        Vector3[] points = new Vector3[count];
        float step = 100 / (count - 1); // Adjust step to distribute points evenly

        for (int i = 0; i < count; i++)
        {
            points[i] = new Vector3(origin.x + i * step, enableY ? Random.Range(-yVariation, yVariation) : origin.y, origin.z);
        }

        return points;
    }

    void OnDrawGizmos()
    {
        if (smoothedPathPoints == null || smoothedPathPoints.Count < 2) return;

        Gizmos.color = Color.cyan;

        for (int i = 0; i < smoothedPathPoints.Count - 1; i++)
        {
            Gizmos.DrawLine(smoothedPathPoints[i], smoothedPathPoints[i + 1]);
        }
    }
}
