using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

public class RL : Agent
{
    public Transform target;
    public Camera agentCamera;
    public Transform PID;

    [Header("RL Params")]
    public float forceMultiplier = 10f;
    public float torqueMultiplier = 100f;  
    public float proximityWeight = 2f;
    public float proximityPenalty = -1f;
    public float fovPenalty = -1f;   
    public float fovLongPenalty = -2f;
    public float fovWeight = 2f;

    [Header("Testing")]
    public bool testing = true;
    public bool disableMove = false;
    public float spawnOffset = 15f;

    [Header("Others")]
    public float maxSpeed = 50f;


    private float bounds = 250;                                                                                                                        
    private double failTime = 0;
    private Vector3 screenCenter;
    private Vector3 screenMin;
    private Vector3 screenMax;
    private Vector3 previousPosition;
    private Vector3 previousBoundingBoxPosition;
    private Vector3 currentBoundingBoxVelocity;


    public override void Initialize()
    {
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
        PathFollower t = target.GetComponent(typeof(PathFollower)) as PathFollower;
        t.Respawn();

        List<int> validPos = new List<int> {-10, -9, -8, -7, -6, 6, 7, 8, 9, 10 };
        Vector3 randomSpawn = new Vector3(validPos[Random.Range(0, validPos.Count)], 0, validPos[Random.Range(0, validPos.Count)]);

        Vector3 targetPosition = target.transform.localPosition;

        transform.localPosition = targetPosition + randomSpawn;
        PID.transform.localPosition = targetPosition + randomSpawn;

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
            AddReward(proximityWeight*(1 - dist/40));

            if(dist > 60 && !testing){
                AddReward(proximityPenalty);
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
            AddReward(fovPenalty);
            if (failTime == 0)
            {
                failTime = DateTime.Now.TimeOfDay.TotalSeconds;
            }
            else if (DateTime.Now.TimeOfDay.TotalSeconds - failTime >= 5)
            {
                Debug.Log ("Out of view");
                AddReward(fovLongPenalty);
                failTime=0;
            }
        }
        else
        {
            failTime = 0;
            float normalizedX = (screenMin.x + screenMax.x) / 2f / Screen.width;
            float normalizedY = (screenMin.y + screenMax.y) / 2f / Screen.height;
            float screenOffset = Mathf.Sqrt(Mathf.Pow(normalizedX - 0.5f, 2) + Mathf.Pow(normalizedY - 0.5f, 2));
    
            AddReward(fovWeight*(1f - screenOffset));
        }
        
        //Alignment reward
        Vector3 targetForward = target.transform.forward.normalized;
        Vector3 agentForward = transform.forward.normalized;
        AddReward(Vector3.Dot(agentForward, targetForward));


        //Penalty for excessive actions
        AddReward(-0.001f);


    }

    void OnTriggerEnter(Collider collision) {
        if(collision.CompareTag("Target") && !testing){
            //Debug.Log("Tagged target");
            AddReward(1f);
        }
    }


    void FixedUpdate() 
    {
	    // episode termination criteria
        if(target.GetComponent<PathFollower>().lapCompleted){
            EndEpisode();
        }

        Vector3 currentVelocity = (transform.localPosition - previousPosition) / Time.deltaTime;
        float speed = currentVelocity.magnitude;
        //Debug.Log("Current speed: " + speed);

        previousPosition = transform.localPosition;
    }
}
