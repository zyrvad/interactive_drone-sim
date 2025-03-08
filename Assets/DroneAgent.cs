using UnityEngine;
using Unity.MLAgents;

public class DroneAgent : Agent
{
    private enum Observables {
        Target,
        Obstacle
    }

    public override void OnEpisodeBegin()
    {
        // spawn drone at random position
        transform.position = new Vector3(Random.Range(-10f, 10f), 10f, Random.Range(-10f, 10f));
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }
}
