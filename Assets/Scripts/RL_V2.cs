using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

public class RL_V2 : Agent
{
    public Transform targetTransform;
    public Camera agentCamera;
    public float bounds = 250; 
    public float maxSpeed = 20f;
    public float forceMultiplier = 150f;
    public float torqueMultiplier = 100f; 
    public LayerMask obstacleLayer;

    [Header("Reward params")]
    public float proximityWeight = 1.0f;
    public float fovWeight = 1.0f;
    public float fovPenalty = -1f;   
    public float fovLongPenalty = -2f;
    public float proximityPenalty = -1.0f;
    public float collisionPenalty = -1.0f;
    public float outofviewPenalty = -1.0f;
    public float crashongroundPenalty = -1.0f;

    private float distanceToTarget;
    private float distanceThreshold = 40f;  // Distance threshold for penalty
    private float desiredDistance = 20f;    // Desired target distance
    private float distanceTolerance = 20f;   // Tolerance range around the desired distance
    private bool targetInDistance = true; 
    private bool targetInView = true; 
    private Rigidbody droneRigidbody;
    private Vector3 previousVelocity;

    private Vector3 screenCenter;
    private Vector3 screenMin;
    private Vector3 screenMax;
    float fovfailTime = 0;
    float distanceFailTime = 0;

    private Vector3 previousBoundingBoxPosition;
    private Vector3 currentBoundingBoxVelocity;

    [SerializeField] private bool obstaclesEnabled = false;
    private TreeDetector treeDetector; // Reference to the TreeDetector script
    private List<Vector4> obstacleBoundingBoxes; // Store bounding boxes from TreeDetector

    // Initialization
    public override void Initialize()
    {
        agentCamera = GetComponent<Camera>();
        droneRigidbody = GetComponent<Rigidbody>();
        treeDetector = GetComponent<TreeDetector>();
        previousVelocity = Vector3.zero;
        previousBoundingBoxPosition = Vector3.zero;
        currentBoundingBoxVelocity = Vector3.zero; 

        // Ensure the Rigidbody has appropriate mass and drag settings
        droneRigidbody.mass = 1f;
        droneRigidbody.linearDamping = 0.5f;
        droneRigidbody.angularDamping = 0.5f;
    }

    // Reset environment at the start of an episode
    public override void OnEpisodeBegin()
    {
        List<int> validPos = new List<int> {-10, -9, -8, -7, -6, 6, 7, 8, 9, 10 };
        transform.localPosition = targetTransform.transform.localPosition + new Vector3(validPos[Random.Range(0, validPos.Count)], 0, validPos[Random.Range(0, validPos.Count)]);

        Vector3 targetPosition = targetTransform.transform.localPosition;
        transform.LookAt(targetPosition);

        previousVelocity = Vector3.zero;
        fovfailTime = 0;
        distanceFailTime = 0;
    }

    // Collect observations
    public override void CollectObservations(VectorSensor sensor)
    {
        // Add agent position and forward direction
        sensor.AddObservation(this.transform.localPosition.x / bounds);
        sensor.AddObservation(this.transform.localPosition.z / bounds);                      
        sensor.AddObservation(this.transform.forward);

        // Add velocity observation
        sensor.AddObservation(droneRigidbody.linearVelocity.normalized); 

        // Add target bounding box observations
        Renderer targetRenderer = targetTransform.GetComponent<Renderer>();
        Bounds targetBounds = targetRenderer.bounds;
        Vector3 minScreen = agentCamera.WorldToScreenPoint(targetBounds.min);
        Vector3 maxScreen = agentCamera.WorldToScreenPoint(targetBounds.max);

        float width = Mathf.Abs(maxScreen.x - minScreen.x);
        float height = Mathf.Abs(maxScreen.y - minScreen.y);

        float normalizedX = minScreen.x / Screen.width;
        float normalizedY = minScreen.y / Screen.height;
        float normalizedWidth = width / Screen.width;
        float normalizedHeight = height / Screen.height;

        bool visible = maxScreen.z > 0 && minScreen.x > 0 && minScreen.y > 0 && minScreen.y < Screen.height;
        if (visible)
        {
            sensor.AddObservation(normalizedX);
            sensor.AddObservation(normalizedY);
            sensor.AddObservation(normalizedWidth);
            sensor.AddObservation(normalizedHeight);

            Vector3 currentBoundingBoxPosition = new Vector3((minScreen.x + maxScreen.x) / 2f, (minScreen.y + maxScreen.y) / 2f, 0);
            currentBoundingBoxVelocity = (currentBoundingBoxPosition - previousBoundingBoxPosition) / Time.deltaTime;
            previousBoundingBoxPosition = currentBoundingBoxPosition;

            sensor.AddObservation(currentBoundingBoxVelocity.x / Screen.width);
            sensor.AddObservation(currentBoundingBoxVelocity.y / Screen.height);
        }
        else
        {
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // Add obstacle bounding boxes (if enabled)
        if (obstaclesEnabled)
        {
            if (treeDetector != null)
            {
                obstacleBoundingBoxes = treeDetector.GetTreeBoundingBoxes();
                foreach (Vector4 bbox in obstacleBoundingBoxes)
                {
                    sensor.AddObservation(bbox.x);
                    sensor.AddObservation(bbox.y);
                    sensor.AddObservation(bbox.z);
                    sensor.AddObservation(bbox.w);
                }
            }
        }
    }

    // Agent actions
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Map continuous actions to agent drone control
        float thrust = actions.ContinuousActions[0]; // Up/down (y-axis)
        float pitch = actions.ContinuousActions[1];  // Forward/backward (x-axis)
        float roll = actions.ContinuousActions[2];   // Left/right (z-axis)
        float yaw = actions.ContinuousActions[3];    // Rotation (yaw)

        // Add noise for exploration
        float noiseScale = 10f; // Adjust as needed
        thrust += Random.Range(-noiseScale, noiseScale);
        pitch += Random.Range(-noiseScale, noiseScale);
        roll += Random.Range(-noiseScale, noiseScale);
        yaw += Random.Range(-noiseScale, noiseScale);

        // Clamp actions to realistic limits
        /* thrust = Mathf.Clamp(thrust, -maxSpeed, maxSpeed);
        pitch = Mathf.Clamp(pitch, -maxSpeed, maxSpeed);
        roll = Mathf.Clamp(roll, -maxSpeed, maxSpeed);
        */
        yaw = Mathf.Clamp(yaw, -1f, 1f); 

        // Apply thrust, pitch, and roll to the agent's Rigidbody
        Vector3 movement = transform.TransformDirection(new Vector3(pitch, thrust, roll)) * forceMultiplier * Time.deltaTime;
        droneRigidbody.AddForce(movement, ForceMode.Acceleration);
        
        float x = Mathf.Clamp(droneRigidbody.linearVelocity.x, -maxSpeed, maxSpeed);
        float y = Mathf.Clamp(droneRigidbody.linearVelocity.y, -maxSpeed, maxSpeed);
        float z = Mathf.Clamp(droneRigidbody.linearVelocity.z, -maxSpeed, maxSpeed);
        droneRigidbody.linearVelocity = new Vector3(x,y,z);

        // Apply yaw rotation
        transform.Rotate(Vector3.up, yaw * torqueMultiplier * Time.deltaTime);

        // Calculate rewards
        CalculateRewards();

        // Check if the target has completed its path
        if (targetTransform.GetComponent<MovementScript>().completedPath)
        {
            targetTransform.GetComponent<MovementScript>().completedPath = false;
            EndEpisode();
        }
    }

    private void CalculateRewards()
    {
        // Proximity reward
        distanceToTarget = Vector3.Distance(transform.localPosition, targetTransform.transform.localPosition);
        AddReward(proximityWeight*(1 - distanceToTarget/distanceThreshold));
        
        // Penalize if the agent is too far or too close
        /*  if (distanceToTarget > distanceThreshold)
        {
            distanceFailTime += Time.deltaTime;
            AddReward(proximityPenalty * distanceFailTime); // Incremental penalty over time
        }
        else
        {
            distanceFailTime = 0f; // Reset if within the desired range
        } */
        
        // Field of View (check if target is in view)
        Renderer targetRenderer = targetTransform.GetComponent<Renderer>();
        Bounds targetBounds = targetRenderer.bounds;
        screenMin = agentCamera.WorldToScreenPoint(targetBounds.min);
        screenMax = agentCamera.WorldToScreenPoint(targetBounds.max);
        screenCenter = (screenMin + screenMax) / 2f;

        targetInView = screenMax.z > 0 && screenMin.x > 0 && screenMin.y > 0 && screenMin.x < Screen.width && screenMin.y < Screen.height;
    
        if (targetInView)
        {
            fovfailTime = 0;
            float normalizedX = (screenMin.x + screenMax.x) / 2f / Screen.width;
            float normalizedY = (screenMin.y + screenMax.y) / 2f / Screen.height;
            float screenOffset = Mathf.Sqrt(Mathf.Pow(normalizedX - 0.5f, 2) + Mathf.Pow(normalizedY - 0.5f, 2));

            float fovReward = fovWeight * (1f - screenOffset);
            AddReward(fovReward);
        }
        else
        {
            fovfailTime += Time.deltaTime;
            AddReward(outofviewPenalty * fovfailTime); // Incremental penalty
        }

        // Alignment reward
        Vector3 targetForward = targetTransform.transform.forward.normalized;
        Vector3 agentForward = transform.forward.normalized;
        AddReward(Vector3.Dot(agentForward, targetForward) * 0.1f); // Scale the alignment reward

        // Collision penalty
        if (Physics.CheckSphere(transform.position, 1f, obstacleLayer))
        {
            Debug.Log("Collision");
            AddReward(collisionPenalty);
            //EndEpisode(); // End episode on collision
        }

        if (transform.localPosition.y <= 0)
        {
            Debug.Log("Crash on ground");
            AddReward(crashongroundPenalty);
            //EndEpisode(); // End episode on ground crash
        }

        AddReward(-0.001f); // Small time penalty to encourage faster learning
    }

    // Debugging rewards (optional)
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, targetTransform.position);

        // Draw a wire sphere at the position of the current object
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, 0.5f);

        // Get the Renderer component of the target object
        Renderer targetRenderer = targetTransform.GetComponent<Renderer>();

        if (targetRenderer != null)
        {
            // Get the world-space bounds of the target object
            Bounds targetBounds = targetRenderer.bounds;

            // Get the min and max points of the target's bounding box in screen space
            Vector3 minScreen = agentCamera.WorldToScreenPoint(targetBounds.min);
            Vector3 maxScreen = agentCamera.WorldToScreenPoint(targetBounds.max);

            // Ensure the min and max screen points are valid (target is visible in screen space)
            if (minScreen.z > 0 && maxScreen.z > 0)
            {
                // Calculate the corners of the bounding box in world space
                Vector3[] corners = new Vector3[8];
                corners[0] = targetBounds.min;
                corners[1] = new Vector3(targetBounds.min.x, targetBounds.min.y, targetBounds.max.z);
                corners[2] = new Vector3(targetBounds.min.x, targetBounds.max.y, targetBounds.min.z);
                corners[3] = new Vector3(targetBounds.min.x, targetBounds.max.y, targetBounds.max.z);
                corners[4] = new Vector3(targetBounds.max.x, targetBounds.min.y, targetBounds.min.z);
                corners[5] = new Vector3(targetBounds.max.x, targetBounds.min.y, targetBounds.max.z);
                corners[6] = new Vector3(targetBounds.max.x, targetBounds.max.y, targetBounds.min.z);
                corners[7] = targetBounds.max;

                // Draw the bounding box using Gizmos (in world space)
                Gizmos.color = Color.blue;

                // Connect the corners to form the bounding box
                Gizmos.DrawLine(corners[0], corners[1]);
                Gizmos.DrawLine(corners[1], corners[3]);
                Gizmos.DrawLine(corners[3], corners[2]);
                Gizmos.DrawLine(corners[2], corners[0]);

                Gizmos.DrawLine(corners[4], corners[5]);
                Gizmos.DrawLine(corners[5], corners[7]);
                Gizmos.DrawLine(corners[7], corners[6]);
                Gizmos.DrawLine(corners[6], corners[4]);

                Gizmos.DrawLine(corners[0], corners[4]);
                Gizmos.DrawLine(corners[1], corners[5]);
                Gizmos.DrawLine(corners[2], corners[6]);
                Gizmos.DrawLine(corners[3], corners[7]);
            }
        }
    }

    void OnTriggerEnter(Collider collision) {
        if(collision.CompareTag("Target")){
            //Debug.Log("Tagged target");
            AddReward(1f);
        }
    }
}
