using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class AdversarialAgent : Agent
{
    public Transform droneTransform;
    public float arenaSize = 20.0f; // Bounds of the environment

    private Rigidbody adversarialRigidbody;

    // Difficulty-related parameters
    private float maxSpeed = 5.0f;  // Maximum speed cap
    private int movementDimension = 2; // Movement dimensions: 2 for x,z; 3 for x,y,z

    public override void Initialize()
    {
        adversarialRigidbody = GetComponent<Rigidbody>();

        // Retrieve initial parameters from the environment
        UpdateDifficultyParameters();
    }

    public override void OnEpisodeBegin()
    {
        // Reset adversarial agent position and velocity
        transform.localPosition = new Vector3(Random.Range(-10, 10), 1, Random.Range(-10, 10));
        adversarialRigidbody.linearVelocity = Vector3.zero;

        // Update difficulty parameters at the start of each episode
        UpdateDifficultyParameters();
    }

    private void UpdateDifficultyParameters()
    {
        // Use Environment Parameters API to set maxSpeed and movementDimension
        maxSpeed = Academy.Instance.EnvironmentParameters.GetWithDefault("max_speed", 5.0f);
        movementDimension = Mathf.FloorToInt(Academy.Instance.EnvironmentParameters.GetWithDefault("movement_dimension", 2));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add relative position of the drone
        sensor.AddObservation(droneTransform.localPosition - transform.localPosition);

        // Add adversarial agent's velocity
        sensor.AddObservation(adversarialRigidbody.linearVelocity);

        // Add direction to the drone
        sensor.AddObservation((droneTransform.localPosition - transform.localPosition).normalized);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Map continuous actions to adversarial agent's movement
        Vector3 controlSignal = new Vector3(actions.ContinuousActions[0], 0, actions.ContinuousActions[1]);

        // Allow movement in the Y direction if movementDimension == 3
        if (movementDimension == 3)
        {
            controlSignal.y = actions.ContinuousActions[2];
        }

        // Clamp velocity to maxSpeed
        adversarialRigidbody.AddForce(controlSignal * 10.0f);
        
        Vector3 velocity = adversarialRigidbody.linearVelocity;
        if (velocity.magnitude > maxSpeed)
        {
            velocity = velocity.normalized * maxSpeed;
            adversarialRigidbody.linearVelocity = velocity;
        }

        Debug.Log("Current speed: " + adversarialRigidbody.linearVelocity.magnitude);
        CalculateEvasionReward();
    }

    private void CalculateEvasionReward()
    {
        // Distance to the drone
        float distanceToDrone = Vector3.Distance(transform.localPosition, droneTransform.localPosition);

        // Reward for staying far from the drone
        AddReward(distanceToDrone * 0.1f);

        // Penalty if the adversarial agent is too close to the drone
        if (distanceToDrone < 1.5f)
        {
            AddReward(-1.0f);
        }

        // End the episode if the adversarial agent goes out of bounds
        if (Mathf.Abs(transform.localPosition.x) > arenaSize || Mathf.Abs(transform.localPosition.z) > arenaSize)
        {
            SetReward(-1.0f);
            EndEpisode();
            droneTransform.GetComponent<RL_V2>().EndEpisode();
        }
    }
}
