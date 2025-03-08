using UnityEngine;

public class SimpleTargetMovement : MonoBehaviour
{
    public float moveSpeed = 3.0f;       // Speed of target movement
    public float oscillationFrequency = 1.0f; // Frequency of oscillation (how fast it moves)
    public float oscillationAmplitude = 5.0f; // Amplitude of movement (distance covered)
    
    private Vector3 startPosition;
    private float time;

    void Start()
    {
        // Store the starting position of the target
        startPosition = transform.position;
    }

    void Update()
    {
        // transform.Translate(Vector3.up * moveSpeed * Time.deltaTime);
        time += Time.deltaTime * oscillationFrequency;

        // Sinusoidal movement along the x-axis
        float xMovement = Mathf.Sin(time) * oscillationAmplitude;
        
        // Optional: Add vertical movement (up and down on the y-axis)
        float yMovement = Mathf.Cos(time) * oscillationAmplitude / 2f; // Smaller amplitude for y-axis

        // Optional: Add movement in z-direction (if you want a circular path)
        float zMovement = Mathf.Sin(time * 0.5f) * oscillationAmplitude;

        // Apply the movement to the target's position
        transform.position = startPosition + new Vector3(xMovement, yMovement, zMovement);
    }
}
