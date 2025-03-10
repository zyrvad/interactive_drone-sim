using UnityEngine;

public class ManualControl : MonoBehaviour
{
    public float horizontalSpeed = 10f;      // Speed for linear movement
    public float verticalSpeed = 3f;
    public float rotationSpeed = 100f; // Speed for yaw rotation

    void Update()
    {
        HandleMovement();
        HandleRotation();
    }

    void HandleMovement()
    {
        Vector3 moveDirection = Vector3.zero;

        // Forward, Backward, Left, Right (Horizontal Movement)
        if (Input.GetKey(KeyCode.W))
            moveDirection += transform.forward;
        if (Input.GetKey(KeyCode.S))
            moveDirection -= transform.forward;
        if (Input.GetKey(KeyCode.A))
            moveDirection -= transform.right;
        if (Input.GetKey(KeyCode.D))
            moveDirection += transform.right;

        // Up and Down (Vertical Movement)
        if (Input.GetKey(KeyCode.Space))
            moveDirection += transform.up;
        if (Input.GetKey(KeyCode.LeftShift))
            moveDirection -= transform.up;

        // Apply horizontal movement with horizontalSpeed
        Vector3 horizontalMovement = new Vector3(moveDirection.x, 0, moveDirection.z).normalized * horizontalSpeed * Time.deltaTime;

        // Apply vertical movement with verticalSpeed
        Vector3 verticalMovement = new Vector3(0, moveDirection.y, 0).normalized * verticalSpeed * Time.deltaTime;

        // Apply both horizontal and vertical movement
        transform.position += horizontalMovement + verticalMovement;
    }

    void HandleRotation()
    {
        // Rotate yaw (left/right) using mouse or keyboard
        if (Input.GetMouseButton(1)) // Right mouse button for mouse control
        {
            float yaw = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            transform.Rotate(Vector3.up, yaw, Space.World);
        }

        // Keyboard controls for yaw rotation
        if (Input.GetKey(KeyCode.LeftArrow))
            transform.Rotate(Vector3.up, -rotationSpeed * Time.deltaTime, Space.World);
        if (Input.GetKey(KeyCode.RightArrow))
            transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime, Space.World);
    }
} 