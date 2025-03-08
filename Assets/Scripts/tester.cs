using UnityEngine;

public class tester : MonoBehaviour
{
    public Transform target;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log($"Right: {transform.right}");
        // Debug.Log($"Forward: {transform.forward}");
        // Debug.Log($"Up: {transform.up}");

        // Vector3 directionToTarget = target.position - transform.position;
        // directionToTarget.y = 0;
        // float yaw = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        transform.Rotate(-Vector3.up, 90 * Time.deltaTime);

        // Debug.Log(yaw);
    }
}
