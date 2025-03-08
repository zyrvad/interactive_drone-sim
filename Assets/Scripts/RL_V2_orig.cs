using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;
using System.Collections.Generic;

public class RL_V2_orig : Agent
{
    public Transform targetTransform;
    public Camera agentCamera;
    public float optimalDistance = 5;
    public float FOV = 80;
    public float bounds = 250; 
    public float maxSpeed = 10;
    public LayerMask obstacleLayer;

    [Header("Reward params")]
    public float proximityWeight = 1.0f;
    public float fovReward = 10.0f;
    public float fovPenalty = 20.0f;
    public float smoothnessWeight = 0.5f;
    public float collisionPenalty = 50.0f;
    public float obstaclePenalty = 10.0f;

    private Rigidbody droneRigidbody;
    private Vector3 previousVelocity;

    private Vector3 screenCenter;
    private Vector3 screenMin;
    private Vector3 screenMax;
    float failTime = 0;

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

        transform.localPosition = targetTransform.localPosition - new Vector3(15,0,0);
    }

    // Reset environment at the start of an episode
    public override void OnEpisodeBegin()
    {
        transform.localPosition = targetTransform.localPosition - new Vector3(15,0,0);
        //transform.localPosition = new Vector3(Random.Range(-10, 10), 1, Random.Range(-10, 10));

        Vector3 targetPosition = targetTransform.localPosition;
        transform.LookAt(targetPosition);
        droneRigidbody.linearVelocity = Vector3.zero;
        droneRigidbody.angularVelocity = Vector3.zero;
        previousVelocity = Vector3.zero;
    }

    // Collect observations
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(this.transform.localPosition.x / bounds);
        sensor.AddObservation(this.transform.localPosition.z / bounds);                      
        sensor.AddObservation(this.transform.forward);
        sensor.AddObservation(droneRigidbody.linearVelocity);

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

        if(obstaclesEnabled){
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
        // Map continuous actions to drone control (e.g., velocity in x, y, z)
        Vector3 controlSignal = new Vector3(actions.ContinuousActions[0], actions.ContinuousActions[1], actions.ContinuousActions[2]) * 100;
        transform.position += transform.right * Mathf.Clamp(controlSignal.z, -maxSpeed, maxSpeed) * Time.deltaTime;
        transform.position += transform.forward * Mathf.Clamp(controlSignal.x, -maxSpeed, maxSpeed) * Time.deltaTime;
        transform.position += transform.up * Mathf.Clamp(controlSignal.y, -maxSpeed, maxSpeed) * Time.deltaTime;

        // Yaw rotation
        float yaw = actions.ContinuousActions[3];
        transform.Rotate(Vector3.up, Mathf.Clamp(yaw*250,-90,90) * Time.deltaTime);

        // Calculate rewards
        CalculateRewards();
    }

    private void CalculateRewards()
    {
        // Distance to target
        float distanceToTarget = Vector3.Distance(transform.localPosition, targetTransform.localPosition);
        float proximityReward = -proximityWeight * Mathf.Pow(distanceToTarget - optimalDistance, 2);
        if(distanceToTarget > 60 ){
            Debug.Log("Target too far away");
            AddReward(-50f);
            EndEpisode();
        }

        // Field of View (check if target is in view)
        Renderer targetRenderer = targetTransform.GetComponent<Renderer>();
        Bounds targetBounds = targetRenderer.bounds;
        screenMin = agentCamera.WorldToScreenPoint(targetBounds.min);
        screenMax = agentCamera.WorldToScreenPoint(targetBounds.max);
        screenCenter = (screenMin + screenMax) / 2f;
    
        bool targetInView = screenMax.z > 0 && screenMin.x > 0 && screenMin.y > 0 && screenMin.x < Screen.width && screenMin.y < Screen.height;
        float fovRewardValue = fovPenalty;
        if(!targetInView){
            if (failTime != 0 && Time.time - failTime >= 5)
            {
                Debug.Log("Out of view");
                AddReward(-50f);
                EndEpisode();
                //targetTransform.GetComponent<AdversarialAgent>().EndEpisode();
            }
            else if (failTime == 0)
            {
                failTime = Time.time;
            }
        }
        else{
            failTime = 0;
            float normalizedX = (screenMin.x + screenMax.x) / 2f / Screen.width;
            float normalizedY = (screenMin.y + screenMax.y) / 2f / Screen.height;
            float screenOffset = Mathf.Sqrt(Mathf.Pow(normalizedX - 0.5f, 2) + Mathf.Pow(normalizedY - 0.5f, 2));
    
            fovRewardValue = fovReward*(1f - screenOffset);
        }

        // Smoothness reward
        Vector3 velocityChange = droneRigidbody.linearVelocity - previousVelocity;
        float smoothnessPenalty = -smoothnessWeight * velocityChange.sqrMagnitude;

        // Collision penalty
        if (Physics.CheckSphere(transform.position, 1f, obstacleLayer))
        {
            Debug.Log("Collision");
            AddReward(collisionPenalty);
            EndEpisode();
            //targetTransform.GetComponent<AdversarialAgent>().EndEpisode();
        }

        // Obstacle proximity penalty
        float obstacleProximityPenalty = 0.0f;
        if (Physics.Raycast(transform.position, transform.forward, out RaycastHit hit, 5.0f, obstacleLayer))
        {
            obstacleProximityPenalty = -obstaclePenalty / hit.distance;
        }

        if(transform.localPosition.y <= 0){
            AddReward(-1000f);
            transform.localPosition = targetTransform.localPosition + new Vector3(Random.Range(-15, 15), targetTransform.localPosition.y, Random.Range(-15, 15));
        }

        // Total reward
        float totalReward = proximityReward + fovRewardValue + smoothnessPenalty + obstacleProximityPenalty;

        // Assign reward
        AddReward(totalReward);

        // Update previous velocity
        previousVelocity = droneRigidbody.linearVelocity;
    }

    // Debugging rewards (optional)
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, targetTransform.position);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, 0.5f);
    }
}
