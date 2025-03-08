using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PIDScreen : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Camera agentCamera;
    [Header("PID Params")]
    [SerializeField] private float trackingOffset = 20f;

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

    [Header("Repulsion Force Params")]
    [SerializeField] private bool obstaclesEnabled = false;
    [SerializeField] private float repulsionRange = 10.0f; // Maximum range for repulsion to apply
    [SerializeField] private float repulsionStrength = 5.0f; // Multiplier for repulsion force
    private TreeDetector treeDetector; // Reference to the TreeDetector script
    private List<Vector4> obstacleBoundingBoxes; // Store bounding boxes from TreeDetector


    private float[] VMAX;
    private Rigidbody rb;
    private readonly float[,] K = new float[3, 4]
    {
        { 0.32f, 0.28f, 0.28f, 0.5f },  // KP (yaw, x, y, z)
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

    // Start is called before the first execution of Update after the MonoBehaviour is created
    public void Start()
    {
        treeDetector = transform.GetComponent<TreeDetector>();
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
        if (!isActiveAndEnabled || agentCamera == null || target == null) return;

        E_n2 = E_n1;
        E_n1 = E;

        float T = Time.deltaTime;

        // Calculate the screen-space position of the target
        Renderer targetRenderer = target.GetComponent<Renderer>();
        if (targetRenderer == null) return;

        Bounds targetBounds = targetRenderer.bounds;
        Vector3 minScreen = agentCamera.WorldToScreenPoint(targetBounds.min);
        Vector3 maxScreen = agentCamera.WorldToScreenPoint(targetBounds.max);

        float screenCenterX = Screen.width / 2f;
        float screenCenterY = Screen.height / 2f;

        float targetCenterX = (minScreen.x + maxScreen.x) / 2f;
        float targetCenterY = (minScreen.y + maxScreen.y) / 2f;

        // Check visibility
        bool visible = maxScreen.z > 0 && minScreen.x > 0 && minScreen.y > 0 && minScreen.y < Screen.height;
        if (!visible) return;

        if(obstaclesEnabled){
            if (treeDetector != null)
            {
                obstacleBoundingBoxes = treeDetector.GetTreeBoundingBoxes();
            }

            foreach (Vector4 bbox in obstacleBoundingBoxes)
            {
                // Calculate screen-space distance from the center of the bounding box to the screen center
                float bboxCenterX = bbox.x + bbox.z / 2f;
                float bboxCenterY = bbox.y + bbox.w / 2f;

                float dx = (Screen.width / 2f - bboxCenterX) / Screen.width;
                float dz = 1.0f / (Mathf.Max(bbox.z, 0.1f)); // Closer objects have stronger repulsion

                // Apply repulsion to x (left/right) and z (forward/backward) errors
                E[1] -= repulsionStrength * dx;
                E[3] += repulsionStrength * dz;
            }

        }

        // Normalize screen-space errors relative to screen center
        float normalizedX = (targetCenterX - screenCenterX) / screenCenterX; // X error
        float normalizedY = (targetCenterY - screenCenterY) / screenCenterY; // Y error

        // Calculate errors
        E = new float[] { (normalizedX-0.5f)/0.5f * 70f, normalizedX, normalizedY, maxScreen.z - trackingOffset };

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
        
        // Move the drone
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
