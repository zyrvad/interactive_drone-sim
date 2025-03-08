using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class PathGenerator : MonoBehaviour
{
    [SerializeField] private int pointsCount = 10;
    [SerializeField] private float bounding = 10f; //half of the length of bounding sqaure
    [SerializeField] private float yVariation = 5f;
    [SerializeField] private int resolution = 10;

    [SerializeField] private PathFollower follower;

    [SerializeField] private bool enableY = false;
    private Vector3[] pathPoints;
    private List<Vector3> smoothedPathPoints;
    public enum PathType {FigureEight, Spiral, ZigZag, SharpTurn, Line, Circle, Random, Star, SPath }
    public PathType selectedPath;
    public enum Size {Small, Medium, Large};
    public Size selectedSize;
    public void Start()
    {
        switch(selectedSize)
        {
            case Size.Small:
                bounding = 25;
                break;
            case Size.Medium:
                bounding = 50;
                break;
            case Size.Large:
                bounding = 100;
                break;
        }
        GenerateAndAssignPaths();
    }
    public void GenerateAndAssignPaths()
    {
        switch (selectedPath)
        {
            case PathType.FigureEight:
                pathPoints = GenerateFigureEightPath(transform.position, bounding);
                break;
            case PathType.Spiral:
                pathPoints = GenerateSpiralPath(transform.position, bounding, 3, resolution);
                break;
            case PathType.ZigZag:
                pathPoints = GenerateZigZagPath(transform.position, (int)bounding/10);
                break;
            case PathType.SharpTurn:
                pathPoints = GenerateSharpTurnPath(transform.position, 100f, 2, 175f);
                break;
            case PathType.Line:
                pathPoints = GenerateStraightLinePath(pointsCount, transform.position, bounding);
                break;
            case PathType.Circle:
                pathPoints = GenerateSmoothCirclePath(transform.position, bounding, resolution);
                break;
            case PathType.Random:
                pathPoints = GenerateTrulyRandomPath(transform.position, pointsCount, bounding);
                break;
            case PathType.Star:
                pathPoints = GenerateStarPath(transform.position, bounding);
                break;
            case PathType.SPath:
                pathPoints = GenerateSPath(transform.position, bounding, resolution);
                break;
        }
        smoothedPathPoints = pathPoints.ToList();
        follower.StartFollowing(smoothedPathPoints);
    }


    Vector3[] GenerateStarPath(Vector3 origin, float scale)
    {
        List<Vector3> starPoints = new List<Vector3>();
        int points = 5; // Five-pointed star
        float angleStep = 360f / points;
    
        // Generate the vertices (outer points only)
        for (int i = 0; i < points; i++)
        {
            float angle = (i * angleStep) * Mathf.Deg2Rad;
            starPoints.Add(new Vector3(
                Mathf.Cos(angle) * scale,
                0,
                Mathf.Sin(angle) * scale
            ) + origin);
        }
    
        // Reorder the points for the pentagram pattern
        List<Vector3> starPath = new List<Vector3>();
        int current = 0;
        do
        {
            starPath.Add(starPoints[current]); // Add the current point
            current = (current + 2) % points; // Skip one point and move to the next
        } while (current != 0); // Stop when we return to the starting point
    
        // Close the loop
        starPath.Add(starPath[0]);
    
        return starPath.ToArray();
    }


    Vector3[] GenerateSPath(Vector3 origin, float scale, int resolution)
    {
        List<Vector3> sPathPoints = new List<Vector3>();

        for (int i = 0; i <= resolution; i++)
        {
            float t = (float)i / resolution; // Normalized position along the path (0 to 1)
            float x = Mathf.Lerp(-scale, scale, t);
            float z = Mathf.Sin(t * Mathf.PI * 2) * scale; // Create the S shape in the Z direction
            float y = enableY ? Random.Range(-yVariation, yVariation) : 0; // Random Y variation if enabled

            sPathPoints.Add(new Vector3(x, y, z) + origin);
        }

        return sPathPoints.ToArray();
    }


    Vector3[] GenerateTrulyRandomPath(Vector3 origin, int pointsCount, float bounding)
    {
        List<Vector3> randomPoints = new List<Vector3>();

        // Generate random points within the bounding box
        for (int i = 0; i < pointsCount; i++)
        {
            float x = Random.Range(-bounding, bounding);
            float z = Random.Range(-bounding, bounding);
            float y = enableY ? Random.Range(-yVariation, yVariation) : 0; // Optional Y variation

            randomPoints.Add(new Vector3(x, y, z) + origin);
        }

        // Ensure the path is closed (optional, remove if not needed)
        randomPoints.Add(randomPoints[0]);

        return randomPoints.ToArray();
    }

    
    Vector3[] GenerateFigureEightPath(Vector3 origin, float scale, int resolution = 100)
    {
        List<Vector3> points = new List<Vector3>();

        for (int i = 0; i <= resolution; i++)
        {
            float t = (float)i / resolution * 2 * Mathf.PI; // Angle from 0 to 2π
            float x = Mathf.Sin(t) * scale;                // X oscillates sinusoidally
            float z = Mathf.Sin(t) * Mathf.Cos(t) * scale; // Z forms two loops (product of sin and cos)

            points.Add(new Vector3(x, 0, z) + origin);
        }

        return points.ToArray();
    }

    //
    Vector3[] GenerateSpiralPath(Vector3 origin, float bounding, float turns, int resolution)
    {
        List<Vector3> points = new List<Vector3>();
        float angleStep = (2 * Mathf.PI * turns) / resolution;

        for (int i = 0; i <= resolution; i++)
        {
            float angle = i * angleStep;
            float currentbounding = Mathf.Lerp(0, bounding, (float)i / resolution);
            points.Add(new Vector3(
                Mathf.Cos(angle) * currentbounding,
                0,
                Mathf.Sin(angle) * currentbounding
            ) + origin);
        }

        return points.ToArray();
    }

    Vector3[] GenerateZigZagPath(Vector3 origin, int segments)
    {
        List<Vector3> points = new List<Vector3>();
        bool left = true;

        // Varying the segment length based on the bounding value
        float segmentLength = bounding * 0.1f;  // Adjust the multiplier as needed (higher values = longer segments)

        // Varying the zigzag width based on the bounding value
        float zigZagWidth = bounding * 0.5f;  // Adjust the multiplier as needed (higher values = wider zigzag)

        // Generate the zigzag path
        for (int i = 0; i < segments; i++)
        {
            points.Add(origin + new Vector3(
                left ? -zigZagWidth : zigZagWidth,  // Alternates between left and right based on the width
                0,
                i * segmentLength  // Moves forward along the Z-axis by segmentLength
            ));
            left = !left;  // Alternates the zigzag direction
        }

        return points.ToArray();
    }

    Vector3[] GenerateSharpTurnPath(Vector3 origin, float segmentLength, int turnCount, float turnAngle)
    {
        Vector3[] points = new Vector3[turnCount + 1]; // +1 to include the start point
        Vector3 currentPosition = origin;
        Vector3 currentDirection = Vector3.forward; // Start moving forward

        points[0] = currentPosition; // First point is the origin

        for (int i = 1; i <= turnCount; i++)
        {
            // Move in the current direction for the segment length
            currentPosition += currentDirection * segmentLength;
            points[i] = currentPosition;

            // Rotate the direction vector by the turn angle
            currentDirection = Quaternion.Euler(0, turnAngle, 0) * currentDirection;
        }

        return points;
    }



    Vector3[] GenerateStraightLinePath(int count, Vector3 origin, float scale)
    {
        Vector3[] points = new Vector3[count];
        float step = scale / (count - 1); // Adjust step to distribute points evenly

        for (int i = 0; i < count; i++)
        {
            points[i] = new Vector3(origin.x + i * step, enableY ? Random.Range(-yVariation, yVariation) : origin.y, origin.z);
        }

        return points;
    }

    Vector3[] GenerateSmoothCirclePath(Vector3 origin, float scale, int resolution)
    {
        Vector3[] points = new Vector3[resolution];
        float angleStep = 2 * Mathf.PI / resolution; // Full circle divided into resolution steps

        for (int i = 0; i < resolution; i++)
        {
            float angle = i * angleStep;
            points[i] = new Vector3(
                Mathf.Cos(angle) * scale,
                0,
                Mathf.Sin(angle) * scale
            ) + origin;
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
