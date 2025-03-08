using UnityEngine;

public class PIDController : MonoBehaviour
{
    [SerializeField] private Transform target;
    [Header("PID Params")]
    [SerializeField] private float trackingOffset = 30f;
    [Header("PID Maximum Speeds")]
    [SerializeField][Tooltip("In deg/s")] private float yaw = 90.0f; // Maximum yaw speed in degrees per second
    [SerializeField] private float right = 6.0f; // Maximum speed to the right
    [SerializeField] private float down = 6.0f; // Maximum speed downwards
    [SerializeField] private float forward = 10.0f; // Maximum speed forwards
    [Header("Enable Directions")]
    [SerializeField] private bool enableYawMovement = true;
    [SerializeField] private bool enableLeftRightMovement = false;
    [SerializeField] private bool enableUpDownMovement = true;
    [SerializeField] private bool enableForwardMovement = true;

    private float[] VMAX;
    private Rigidbody rb;
    private readonly float[,] K = new float[3, 4]
    {
        { 0.32f, 0.28f, 0.28f, 0.28f },  // KP (yaw, x, y, z)
        { 0.025f, 0.0384f, 0.0384f, 0.0384f },  // KI (yaw, x, y, z)
        { 0.0f, 0.0f, 0.0f, 0.0f }  // KD (yaw, x, y, z)
    };
    private enum KParameter
    {
        KP,
        KI,
        KD
    }
    private float[] U_n1 = new float[4]; // [yaw, x, y, z] => signals
    private float[] E = new float[4]; // [yaw, x, y, z]
    private float[] E_n1 = new float[4]; // [yaw, x, y, z]
    private float[] E_n2 = new float[4]; // [yaw, x, y, z]

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        VMAX = new float[] { yaw, right, down, forward };
    }

    // OnDrawGizmos for visual debugging of the drone's forward direction and target
    private void OnDrawGizmos()
    {
        // Draw the front direction of the drone
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, transform.forward * 5f); // Forward arrow

        // Draw line to the target
        if (target != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position); // Line to target
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!isActiveAndEnabled) return;

        E_n2 = E_n1;
        E_n1 = E;

        float T = Time.deltaTime;

        // Calculate the error
        Vector3 worldDirectionToTarget = target.position - transform.position;
        Vector3 relativeDirectionToTarget = transform.InverseTransformDirection(worldDirectionToTarget);
        float x = relativeDirectionToTarget.x;
        float y = relativeDirectionToTarget.y;
        float z = relativeDirectionToTarget.z - trackingOffset;
        worldDirectionToTarget.y = 0;
        float yaw = Vector3.SignedAngle(transform.forward, worldDirectionToTarget, Vector3.up);

        E = new float[] { yaw, x, y, z };

        float[] U = new float[4]; // [yaw, x, y, z] => signals
        for (int i = 0; i <= 3; i++)
        {
            U[i] = Mathf.Clamp(U_n1[i] + 
                E[i] * (K[(int)KParameter.KP, i] + K[(int)KParameter.KI, i] * T + K[(int)KParameter.KD, i] / T) -
                E_n1[i] * (K[(int)KParameter.KP, i] + 2 * K[(int)KParameter.KD, i] / T) +
                E_n2[i] * K[(int)KParameter.KD, i] / T, -VMAX[i], VMAX[i]);
        }

        // Update the previous signals
        U_n1 = U;
        
        // move the drone
        if (enableYawMovement) transform.Rotate(-Vector3.up, -U[0] * Time.deltaTime);
        if (enableLeftRightMovement) transform.position += transform.right * U[1] * Time.deltaTime;
        if (enableUpDownMovement) transform.position += transform.up * U[2] * Time.deltaTime;
        if (enableForwardMovement) transform.position += transform.forward * U[3] * Time.deltaTime;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("[PID] Hit an obstacle!");
        }
    }
}