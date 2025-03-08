using System.Collections.Generic;
using UnityEngine;

public class PerlinTreeGenerator : MonoBehaviour
{
    [Header("Drones")]
    [SerializeField] private MovementScript enemyDrone;
    [SerializeField] private GameObject pIDDrone;
    [SerializeField] private GameObject rLDrone;
    [Header("Environment Settings")]
    [SerializeField] private SpawnObjects spawnObjects = SpawnObjects.TargetOnly;
    [SerializeField] private GameObject[] treePrefabs;
    [SerializeField] private Grid environmentGrid;
    [SerializeField] private int width = 500;
    [SerializeField] private int depth = 500;
    [SerializeField] private float scale = 0.1f;
    [SerializeField] private float threshold = 0.5f;
    [Header("Spawn Settings")]
    public float minDroneHeight = 10f;
    public float maxDroneHeight = 20f;
    public float spawnRadius = 20f; // No trees will spawn within this radius of the drone
    private GameObject treeParent;
    private Vector3 targetSpawnPosition;
    private Vector3 targetEndPosition;
    private Vector3 droneSpawnPosition;

    private enum SpawnObjects
    {
        TreesOnly,
        TargetOnly,
        PIDAndTarget,
        RLAndTarget,
        All
    }


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        treeParent = new GameObject("Trees");
        treeParent.transform.SetParent(transform.parent);

        switch (spawnObjects)
        {
            case SpawnObjects.TreesOnly:
                GenerateForest();
                break;
            case SpawnObjects.TargetOnly:
                targetSpawnPosition = RandomSpawnPosition();
                targetEndPosition = RandomSpawnPosition();

                GenerateForest();
                environmentGrid.GenerateGrid(width, depth, height: 30);

                // Visualize the end node
                if (environmentGrid.showEndNode)
                {
                    Node endNode = environmentGrid.GetNodeFromWorldPosition(targetEndPosition);
                    environmentGrid.VisualizeGridCell(endNode.position, Grid.GridCellType.End);
                }

                SpawnDrone(enemyDrone.gameObject, targetSpawnPosition);
                enemyDrone.PerformAStar(targetSpawnPosition, targetEndPosition);
                break;
            case SpawnObjects.PIDAndTarget:
                targetSpawnPosition = RandomSpawnPosition();
                targetEndPosition = RandomSpawnPosition();
                droneSpawnPosition = new Vector3(targetSpawnPosition.x + 10, targetSpawnPosition.y, targetSpawnPosition.z);

                GenerateForest();
                environmentGrid.GenerateGrid(width, depth, height: 30);

                // Visualize the end node
                if (environmentGrid.showEndNode)
                {
                    Node endNode = environmentGrid.GetNodeFromWorldPosition(targetEndPosition);
                    environmentGrid.VisualizeGridCell(endNode.position, Grid.GridCellType.End);
                }

                SpawnDrone(enemyDrone.gameObject, targetSpawnPosition);
                SpawnDrone(pIDDrone, droneSpawnPosition);

                enemyDrone.PerformAStar(targetSpawnPosition, targetEndPosition);
                break;
            case SpawnObjects.RLAndTarget:
                break;
            case SpawnObjects.All:
                break;
            default:
                break;
        }
    }

    private void GenerateForest()
    {
        int counter = 0;

        // Loop through the grid
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                // Sample Perlin noise at each grid point
                float noiseValue = Mathf.PerlinNoise(x * scale, z * scale);

                // If the noise value is above the threshold, place a tree
                if (noiseValue > threshold)
                {
                    // Get the position where the tree should go
                    Vector3 treePosition = new(x, 0, z);

                    // Skip if the tree is too close to the drone spawn point
                    if (targetSpawnPosition != Vector3.zero && targetEndPosition != Vector3.zero && (
                        Vector3.Distance(treePosition, targetSpawnPosition) <= spawnRadius ||
                        Vector3.Distance(treePosition, targetEndPosition) <= spawnRadius) ||
                        Vector3.Distance(treePosition, droneSpawnPosition) <= spawnRadius)
                    {
                        continue;
                    }

                    // Randomly select a tree prefab from the array
                    int randomTreeIndex = Random.Range(0, treePrefabs.Length);
                    GameObject tree = treePrefabs[randomTreeIndex];

                    // Instantiate the selected tree prefab at this position
                    GameObject treeInstance = Instantiate(tree, treePosition, Quaternion.Euler(0, Random.Range(0, 360), 0));
                    treeInstance.layer = LayerMask.NameToLayer("Obstacle");

                    // Optionally, scale the tree for more variation in height
                    float randomScaleFactor = GetHeightVariation(randomTreeIndex); // Random height scaling
                    treeInstance.transform.localScale = new Vector3(treeInstance.transform.localScale.x, randomScaleFactor, treeInstance.transform.localScale.z);

                    treeInstance.transform.SetParent(treeParent.transform);
                    treeInstance.tag = "Obstacle";
                    treeInstance.name = $"Tree #{counter}";

                    counter++;
                }
            }
        }
    }

    private void SpawnDrone(GameObject obj, Vector3 spawnPosition)
    {
        obj.transform.position = spawnPosition;
        obj.SetActive(true);
    }

    private Vector3 RandomSpawnPosition()
    {
        float spawnX = Random.Range(0, width);
        float spawnZ = Random.Range(0, depth);
        float spawnY = Random.Range(minDroneHeight, maxDroneHeight);

        return new Vector3(spawnX, spawnY, spawnZ);
    }

    private float GetHeightVariation(int treeIndex)
    {
        float randomHeightFactor = 1f;

        // Add height variation based on tree type
        switch (treeIndex)
        {
            case 0: // Bare tree
                randomHeightFactor = Random.Range(0.125f, 0.3f); // Bare trees can be smaller
                break;
            case 1: // Small tree
                randomHeightFactor = Random.Range(0.25f, 0.375f);   // Small trees can be slightly larger
                break;
            case 2: // Medium tree
                randomHeightFactor = Random.Range(0.375f, 0.625f); // Medium trees should be larger
                break;
            case 3: // Large tree
                randomHeightFactor = Random.Range(0.5f, 1f);     // Large trees should be the tallest
                break;
            default:
                break;
        }

        return randomHeightFactor;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
