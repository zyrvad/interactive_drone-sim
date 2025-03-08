using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine.AI;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RL_orig : Agent
{
    public Transform target;
    public Camera agentCamera;
    public Transform PID;

    [Header("RL Params")]
    public float forceMultiplier = 10f;
    public float torqueMultiplier = 100f;     

    [Header("Testing")]
    public bool testing = true;
    public bool disableMove = false;
    public float spawnOffset = 15f;

    float bounds = 250;                                                                                                                        
    float episodeCounter;	
    float failTime = 0;

   
    public float maxSpeed = 50f;
    private bool obstaclesEnabled;
    
    private Vector3 screenCenter;
    private Vector3 screenMin;
    private Vector3 screenMax;
    private Vector3 previousPosition;

    private Vector3 previousBoundingBoxPosition;
    private Vector3 currentBoundingBoxVelocity;
    private float episodeStartTime;
    private bool cap = false;
    

    public override void Initialize()
    {
        episodeCounter += 1;
        previousPosition = transform.localPosition;
        agentCamera = GetComponent<Camera>();
        previousBoundingBoxPosition = Vector3.zero;
        currentBoundingBoxVelocity = Vector3.zero;
    }

    /*
    1. update episode count
    2. reset velocity and angular velocity of my drone
    3. reset position and rotation of my drone
    4. reset velocity of target
    5. random z coorindate for target
    */ 
    public override void OnEpisodeBegin()
    {
        episodeCounter += 1;
        episodeStartTime = Time.time;

        PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
        t.Respawn();

        // Minimum spawn distance from the target
        float minimumDistance = 5f;
        Vector3 randomSpawn;

        // Ensure agent's spawn point is not too close to the target
        do
        {
            randomSpawn = new Vector3(Random.Range(-spawnOffset, spawnOffset), 0, Random.Range(-spawnOffset, spawnOffset));
        } 
        while (Vector3.Distance(target.localPosition + randomSpawn, target.localPosition) < minimumDistance);

        Vector3 targetPosition = target.transform.localPosition;

        if (!disableMove)
        {
            transform.localPosition = target.localPosition + randomSpawn;
        }
        else
        {
            transform.localPosition = new Vector3(50, 0, 30);
        }

        // Ensure PID's spawn point is not too close to the target
        Vector3 pidSpawn;
        do
        {
            pidSpawn = new Vector3(Random.Range(-spawnOffset, spawnOffset), 0, Random.Range(-spawnOffset, spawnOffset));
        } 
        while (Vector3.Distance(target.localPosition + pidSpawn, target.localPosition) < minimumDistance);

        PID.transform.localPosition = target.localPosition + pidSpawn;
        PID.transform.LookAt(targetPosition);

        transform.LookAt(targetPosition);
        previousPosition = transform.localPosition;
        previousBoundingBoxPosition = Vector3.zero;
        currentBoundingBoxVelocity = Vector3.zero;
    }

    /*
    Agent: position, speed, direction
    Target: screen coordinates, camera sensor, speed
    */
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(this.transform.localPosition.x / bounds);
        sensor.AddObservation(this.transform.localPosition.z / bounds);                      
	    sensor.AddObservation(this.transform.forward);
                     
        Renderer targetRenderer = target.GetComponent<Renderer>();
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
        if(visible){
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
        else{
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(-1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
    }

    public override void OnActionReceived(ActionBuffers act)
    {
        Vector3 movement = new Vector3
        {
            x = act.ContinuousActions[0],
            y = act.ContinuousActions[3],
            z = act.ContinuousActions[1]
        } * forceMultiplier;
        
        
        transform.position += transform.right * Mathf.Clamp(movement.z, -maxSpeed, maxSpeed) * Time.deltaTime;
        transform.position += transform.forward * Mathf.Clamp(movement.x, -maxSpeed, maxSpeed) * Time.deltaTime;
        transform.position += transform.up * Mathf.Clamp(movement.y, -maxSpeed, maxSpeed) * Time.deltaTime;


        if(!disableMove){


            float dist = Vector3.Distance(target.transform.localPosition, transform.localPosition);
            AddReward(2*(1 - dist/40));

            if(dist > 60 && !testing){
                AddReward(-50f);
                EndEpisode();
                PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
                t.Respawn();
            }
        }
        
        float rotateY = act.ContinuousActions[2] * torqueMultiplier * Time.deltaTime;
        transform.Rotate(0, rotateY, 0);
    
        Renderer targetRenderer = target.GetComponent<Renderer>();
        Bounds targetBounds = targetRenderer.bounds;
        screenMin = agentCamera.WorldToScreenPoint(targetBounds.min);
        screenMax = agentCamera.WorldToScreenPoint(targetBounds.max);
        screenCenter = (screenMin + screenMax) / 2f;
    
        bool targetVisible = screenMax.z > 0 && screenMin.x > 0 && screenMin.y > 0 && screenMin.x < Screen.width && screenMin.y < Screen.height;
        if (!targetVisible)
        {
            //Debug.Log("Target cannot be seen");
            AddReward(-3f);
            if (failTime != 0)
            {
                if (Time.time - failTime >= 5 && !testing)
                {
                    Debug.Log("Target invisible for too long. Ending episode.");
                    AddReward(-50f);
                    EndEpisode();
                    PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
                    t.Respawn();
                }
            }
            else
            {
                failTime = Time.time;
            }
        }
        else
        {
            failTime = 0;
            float normalizedX = (screenMin.x + screenMax.x) / 2f / Screen.width;
            float normalizedY = (screenMin.y + screenMax.y) / 2f / Screen.height;
            float screenOffset = Mathf.Sqrt(Mathf.Pow(normalizedX - 0.5f, 2) + Mathf.Pow(normalizedY - 0.5f, 2));
    
            AddReward(2*(1f - screenOffset));
        }
    
        AddReward(-0.001f);
        

        Vector3 targetForward = target.transform.forward.normalized;
        Vector3 agentForward = transform.forward.normalized;
        AddReward(10*Vector3.Dot(agentForward, targetForward));
    }

    void OnTriggerEnter(Collider collision) {
        if(collision.CompareTag("Target") && !testing){
            //Debug.Log("Tagged target");
            AddReward(50f);
            EndEpisode();
            PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
            t.Respawn();
        }
    }


    void FixedUpdate() 
    {
	    // episode termination criteria
	    if (Time.time - episodeStartTime > 60f && !testing) 
	    {
	        //EndEpisode();
            //PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
            //t.Respawn();
	    }

        Vector3 currentVelocity = (transform.localPosition - previousPosition) / Time.deltaTime;
        float speed = currentVelocity.magnitude;
        //Debug.Log($"Previous Position: {previousPosition}, Current Position: {transform.localPosition}");
        Debug.Log("Current speed: " + speed);

        previousPosition = transform.localPosition;
    }
}
