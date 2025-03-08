using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class PathFollower : MonoBehaviour
{
    [SerializeField] public float baseSpeed = 12f;             // Base speed of the object
    [SerializeField] public float maxAcceleration = 5f;       // Maximum speed adjustment
    [SerializeField] public bool useStraightPath = false;
    [SerializeField] public bool training = true;
    [SerializeField] private float minTime = 1f;               // Minimum time between random accelerations
    [SerializeField] private float maxTime = 5f;               // Maximum time between random accelerations
    [SerializeField] private float accelerationRate = 1f;      // Rate at which speed adjusts (units/sec)
    private List<Vector3> pathPoints;
    private int currentPointIndex = 0;

    private float currentSpeed;                                // Current speed of the object
    private float targetSpeed;                                 // The speed we are moving toward

    private float randomTimer;                                 // Timer for random acceleration

    [SerializeField] private GameObject generator;
    BoxCollider collider;
    EnvironmentParameters m_ResetParams;
    private Vector3 initialPosition;
    public bool lapCompleted = false;

    private float startTime;

    public void Respawn() {
        if(training){
            baseSpeed = m_ResetParams.GetWithDefault("targetSpeed", 2f);
            maxAcceleration = m_ResetParams.GetWithDefault("acc", 0f);
            float length = m_ResetParams.GetWithDefault("radius", 1f);
            collider.size = new Vector3(length, length, length);
        }
        startTime = Time.time;

        // Reset speed to 0 and set target speed to baseSpeed
        currentSpeed = 0;
        targetSpeed = baseSpeed;

        // Generate new paths
        //RandomPathGenerator rnd = generator.GetComponent<RandomPathGenerator>();
        //rnd.GenerateAndAssignPaths();

        PathGenerator rnd = generator.GetComponent<PathGenerator>();
        rnd.GenerateAndAssignPaths();   
    }

    void Start()
    {
        startTime = Time.time;
        currentSpeed = !useStraightPath ? 0 : baseSpeed;
        targetSpeed = baseSpeed;
        ResetRandomTimer(); // Initialize the random timer

        if(training){
            m_ResetParams = Academy.Instance.EnvironmentParameters;
            baseSpeed = m_ResetParams.GetWithDefault("targetSpeed", 2f);
            maxAcceleration = m_ResetParams.GetWithDefault("acc", 0f);
            collider = this.transform.GetComponent<BoxCollider>();
            float length = m_ResetParams.GetWithDefault("radius", 15f);
            collider.size = new Vector3(length, length, length); 
        }
    }

    public void StartFollowing(List<Vector3> pathPoints)
    {
        this.pathPoints = pathPoints;
        currentPointIndex = 0;

        initialPosition = transform.position; // Save the starting position of the target
        lapCompleted = false;

        if (pathPoints != null && pathPoints.Count > 0)
        {
            transform.position = pathPoints[0];
        }
    }

    void Update()
    {
        if (pathPoints == null || pathPoints.Count < 2) return;

        // Update the random acceleration timer
        randomTimer -= Time.deltaTime;
        if (randomTimer <= 0)
        {
            SetRandomTargetSpeed();
            ResetRandomTimer();
        }

        // Gradually adjust current speed toward target speed
        currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, accelerationRate * Time.deltaTime);
        //Debug.Log("Current speed: " + currentSpeed);

        // Move towards the current path point
        Vector3 target = pathPoints[currentPointIndex];
        transform.position = Vector3.MoveTowards(transform.position, target, currentSpeed * Time.deltaTime);

        // Check if we've reached the target point
        if (Vector3.Distance(transform.position, target) < 0.1f)
        {
            // Increment to the next point
            int previousPointIndex = currentPointIndex; // Store the previous index
            currentPointIndex = (currentPointIndex + 1) % pathPoints.Count;

            // Check if we completed a lap by wrapping from the last point to the first
            if (previousPointIndex == pathPoints.Count - 1)
            {
                lapCompleted = true; // Target has completed one lap
                Debug.Log("Lap completed!");
            }
        }

        // Optional: Rotate the object to face the direction of movement
        Vector3 direction = (target - transform.position).normalized;
        if (direction.magnitude > 0.1f)
        {
            transform.forward = direction;
        }
    }


    private void SetRandomTargetSpeed()
    {
        // Generate a random target speed
        float randomAcceleration = Random.Range(-maxAcceleration, maxAcceleration);
        targetSpeed = Mathf.Clamp(baseSpeed + randomAcceleration, 0f, baseSpeed + maxAcceleration);
        //Debug.Log($"New target speed set: {targetSpeed}");
    }

    private void ResetRandomTimer()
    {
        // Set a new random time between the min and max intervals
        randomTimer = Random.Range(minTime, maxTime);
        // Debug.Log($"Next speed adjustment in: {randomTimer} seconds");
    }
}
