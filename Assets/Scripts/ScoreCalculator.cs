using System.IO;
using UnityEngine;

public class ScoreCalculator : MonoBehaviour
{
    [SerializeField] private float viewDistance = 40f;
    [SerializeField][Range(0, 180)] private float FOV = 60f;
    [SerializeField] private Transform target;
    [Header("Output Settings")]
    [SerializeField] private string outputDirectory = "C:/Users/UserAdmin/Terrain Generation/Assets/DataProcessing";

    private float score = 0f;
    private float totalTime = 0f;
    public float percent = 0f;
    public float timePercent = 0f;

    private Camera cam;
    private float currentOutOfViewTime = 0f; // Current duration the target is out of view
    public float maxOutOfViewTime = 0f;     // Maximum continuous duration the target was out of view

    void Start()
    {
        // Clear the output file at the start
        File.WriteAllText($"{outputDirectory}/{transform.name}.csv", string.Empty);

        cam = GetComponent<Camera>();
        cam.fieldOfView = FOV;
        cam.farClipPlane = viewDistance;
    }

    void Update()
    {
        float dt = Time.deltaTime;
    
        if (IsTargetVisible())
        {
            // Increment score if target is visible
            score += dt;
    
            // Reset the out-of-view timer
            currentOutOfViewTime = 0f;
        }
        else
        {
            // Increment the out-of-view timer if target is not visible
            currentOutOfViewTime += dt;
    
            // Update maxOutOfViewTime every frame if the current out-of-view time is greater
            if (currentOutOfViewTime > maxOutOfViewTime)
            {
                maxOutOfViewTime = currentOutOfViewTime;
            }
        }
    
        totalTime += dt;
        percent = score / totalTime;
        timePercent = maxOutOfViewTime / totalTime; // Update timePercent to reflect maxOutOfViewTime as a percentage of total time
    
        // Log total time, percentage of time target visible, and percentage of time out of view
        Log($"{totalTime},{percent},{timePercent}");
    }
    
    void OnApplicationQuit()
    {
        // Ensure final maximum out-of-view time is set
        if (currentOutOfViewTime > maxOutOfViewTime)
        {
            maxOutOfViewTime = currentOutOfViewTime;
        }
    
        Debug.Log($"Score: {percent}");
        Debug.Log($"Maximum out-of-view time: {maxOutOfViewTime}");
        Debug.Log($"Time Percent: {timePercent}");
    }
    

    private void Log(string line)
    {
        File.AppendAllText($"{outputDirectory}/{transform.name}.csv", line + "\n");
    }

    public void ResetScore()
    {
        score = 0f;
        totalTime = 0f;
        percent = 0f;
        currentOutOfViewTime = 0f;
        maxOutOfViewTime = 0f;
    }

    private bool IsTargetVisible()
    {
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        if (distanceToTarget > viewDistance) return false; // Too far away

        Vector3 screenPos = cam.WorldToScreenPoint(target.position);
        bool isTargetVisible = screenPos.z > 0 &&
            screenPos.x > 0 && screenPos.x < Screen.width &&
            screenPos.y > 0 && screenPos.y < Screen.height;

        return isTargetVisible;
    }
}
