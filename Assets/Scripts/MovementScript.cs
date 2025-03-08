using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.Timeline;
using UnityEngine;
using UnityEngine.AI;

public class MovementScript : MonoBehaviour
{
    [SerializeField] private float liftForce = 9.81f;
    [SerializeField] private float thrustMultiplier = 1000f;
    [SerializeField] private ControlMode controlMode = ControlMode.Manual;
    [SerializeField] private Grid grid;
    public bool completedPath = false;
    private bool firstTime = true;
    private Rigidbody rb;
    private enum ControlMode
    {
        Manual,
        Pathfinding
    }
    Vector3 randomEndPosition;
    Vector3 startPosition;
    // private bool isComputingPath = false; // in the future when i want to multithread pathfinding

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    public void Start()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = new Vector3(250,10,250);
        ChooseNewTarget();
    }

    public void PerformAStar(Vector3 startPosition, Vector3 endPosition)
    {
        if (controlMode != ControlMode.Pathfinding) return;

        AStarPathfinding aStarPathfinding = new(grid);
        List<Node> path = aStarPathfinding.FindPath(startPosition, endPosition);

        if (path != null && path.Count > 0)
        {   
            StartCoroutine(FollowPath(path));
        }
        else
        {
            Debug.Log("No path found! Finding new path...");
            ChooseNewTarget();
        }
    }

    private IEnumerator FollowPath(List<Node> path)
    {
        foreach (Node node in path)
        {
            // Move towards the current node
            while (Vector3.Distance(transform.position, node.position) > 0.1f)
            {
                // Direction to the target
                Vector3 direction = (node.position - transform.position).normalized;

                // Smoothly rotate toward the target
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);

                transform.position = Vector3.MoveTowards(
                    transform.position,
                    node.position,
                    thrustMultiplier * Time.deltaTime // Adjust the speed
                );
                yield return null; // Wait for the next frame
            }
        }

        Debug.Log("Path completed!");
        ChooseNewTarget();
    }

    public void ChooseNewTarget()
    {
        if(firstTime){
            do{
                randomEndPosition = grid.GetRandomWalkableCell().position;
            }while(Vector3.Distance(randomEndPosition, startPosition) > 150 || Vector3.Distance(randomEndPosition, startPosition) < 100);
            firstTime = false;
        }
        
        completedPath = true;

        transform.localPosition = startPosition;
        Debug.Log($"New target: {randomEndPosition}");
        Destroy(GameObject.FindWithTag("End Cell"));
        grid.VisualizeGridCell(randomEndPosition, Grid.GridCellType.End);
        PerformAStar(startPosition, randomEndPosition);
    }

    // Update is called once per frame
    void Update()
    {
    }

    void FixedUpdate()
    {
        // Cap velocity of enemy to 10km/h = 2.77m/s
        if (rb.linearVelocity.magnitude > 8f)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * 8f;
        }

        // Apply upward force to counter gravity
        rb.AddForce(Vector3.up * liftForce, ForceMode.Acceleration);

        if (Input.GetKey(KeyCode.W)) rb.AddForce(transform.forward * thrustMultiplier, ForceMode.Force);
        if (Input.GetKey(KeyCode.S)) rb.AddForce(-transform.forward * thrustMultiplier, ForceMode.Force);
        if (Input.GetKey(KeyCode.A)) rb.AddForce(-transform.right * thrustMultiplier, ForceMode.Force);
        if (Input.GetKey(KeyCode.D)) rb.AddForce(transform.right * thrustMultiplier, ForceMode.Force);
        if (Input.GetKey(KeyCode.Space)) rb.AddForce(transform.up * thrustMultiplier, ForceMode.Force);
        if (Input.GetKey(KeyCode.LeftShift)) rb.AddForce(-transform.up * thrustMultiplier, ForceMode.Force);

        if (Input.GetKey(KeyCode.LeftArrow)) rb.AddTorque(Vector3.down * thrustMultiplier * Time.deltaTime, ForceMode.Force);
        if (Input.GetKey(KeyCode.RightArrow)) rb.AddTorque(Vector3.up * thrustMultiplier * Time.deltaTime, ForceMode.Force);
        // if (Input.GetKey(KeyCode.UpArrow)) rb.AddTorque(Vector3.left * thrustMultiplier, ForceMode.Force);
        // if (Input.GetKey(KeyCode.DownArrow)) rb.AddTorque(Vector3.right * thrustMultiplier, ForceMode.Force);

        if (!Input.GetKey(KeyCode.LeftArrow) && !Input.GetKey(KeyCode.RightArrow))
        {
            // Apply stabilization torque to smooth out rotations
            Vector3 stabilizationTorque = -rb.angularVelocity * 10f * Time.deltaTime;
            rb.AddTorque(stabilizationTorque, ForceMode.Force);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle"))
        {
            Debug.Log($"Hit an obstacle! {other.name}");
        }
    }
}
