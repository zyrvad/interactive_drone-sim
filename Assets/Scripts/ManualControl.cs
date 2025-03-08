using UnityEngine;

public class ManualControl : MonoBehaviour
{
    public float moveSpeed = 10f;      // Speed for linear movement
    public float rotationSpeed = 100f; // Speed for yaw rotation

    void Update()
    {
        HandleMovement();
        HandleRotation();
    }

    void HandleMovement()
    {
        Vector3 moveDirection = Vector3.zero;

        // Forward, Backward, Left, Right
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            moveDirection += transform.forward;
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            moveDirection -= transform.forward;
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            moveDirection -= transform.right;
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            moveDirection += transform.right;

        // Up and Down
        if (Input.GetKey(KeyCode.Q))
            moveDirection += transform.up;
        if (Input.GetKey(KeyCode.E))
            moveDirection -= transform.up;

        // Apply movement
        transform.position += moveDirection * moveSpeed * Time.deltaTime;
    }

    void HandleRotation()
    {
        // Rotate yaw (left/right) using mouse or keyboard
        if (Input.GetMouseButton(1)) // Right mouse button for mouse control
        {
            float yaw = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            transform.Rotate(Vector3.up, yaw, Space.World);
        }

        if (Input.GetKey(KeyCode.LeftArrow))
            transform.Rotate(Vector3.up, -rotationSpeed * Time.deltaTime, Space.World);
        if (Input.GetKey(KeyCode.RightArrow))
            transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime, Space.World);
    }
} 