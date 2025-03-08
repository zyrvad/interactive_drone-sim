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
    private Rigidbody rb;
    private enum ControlMode
    {
        Manual,
        Pathfinding
    }
    // private bool isComputingPath = false; // in the future when i want to multithread pathfinding

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
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

    private void ChooseNewTarget()
    {
        Vector3 randomEndPosition = grid.GetRandomWalkableCell().position;

        Debug.Log($"New target: {randomEndPosition}");
        Destroy(GameObject.FindWithTag("End Cell"));
        grid.VisualizeGridCell(randomEndPosition, Grid.GridCellType.End);
        PerformAStar(transform.position, randomEndPosition);
    }

    // Update is called once per frame
    void Update()
    {
    }

    void FixedUpdate()
    {
        // Cap velocity of enemy to 10km/h = 2.77m/s
        if (rb.linearVelocity.magnitude > 10f)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * 10f;
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
