using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{
    [SerializeField] private Vector3 origin;
    [SerializeField] private float cellSize = 1f;
    [SerializeField] private float obstacleThreshold = 0.5f;
    [SerializeField] private Material walkableMaterial;
    [SerializeField] private Material blockedMaterial;
    public Material endNodeMaterial;
    [SerializeField] private bool showWalkableNodes = false;
    [SerializeField] private bool showObstacleNodes = true;
    public bool showEndNode = false;
    public int gridWidth, gridHeight, gridDepth;
    public enum GridCellType
    {
        Walkable,
        Unwalkable,
        Start,
        End
    }
    private Node[,,] grid;
    private List<Node> walkableNodes = new();
    private List<Node> unwalkableNodes = new();


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        origin = new(cellSize / 2, cellSize / 2, cellSize / 2);
    }

    public void GenerateGrid(int width, int depth, int height)
    {
        gridWidth = Mathf.CeilToInt(width / cellSize);
        gridHeight = Mathf.CeilToInt(height / cellSize);
        gridDepth = Mathf.CeilToInt(depth / cellSize);

        grid = new Node[gridWidth, gridHeight, gridDepth]; // TODO, need to adjust the nodes

        // Loop through each cell in the grid
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                for (int z = 0; z < gridDepth; z++)
                {
                    Vector3 worldPosition = origin + new Vector3(x * cellSize, y * cellSize, z * cellSize);
                    bool isWalkable = !Physics.CheckSphere(worldPosition, cellSize * obstacleThreshold);

                    // Create the node and set its position and walkability
                    Node node = new(worldPosition, isWalkable);

                    // Set the grid coordinates for this node
                    node.SetGridCoordinates(x, y, z);

                    // Store the node in the grid
                    grid[x, y, z] = node;

                    if (isWalkable) walkableNodes.Add(node);
                    else unwalkableNodes.Add(node);

                    // Visualize the grid with a cube
                    if (showWalkableNodes && isWalkable) VisualizeGridCell(worldPosition, GridCellType.Walkable);
                    else if (showObstacleNodes && !isWalkable) VisualizeGridCell(worldPosition, GridCellType.Unwalkable);
                }
            }
        }
    }

    public Node GetRandomWalkableCell() 
    {
        return walkableNodes[Random.Range(0, walkableNodes.Count)];
    }

    public void VisualizeGridCell(Vector3 position, GridCellType cellType)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = position;
        cube.transform.localScale = cellSize * 0.9f * Vector3.one;

        Material material;

        switch (cellType)
        {
            case GridCellType.Walkable:
                material = walkableMaterial;
                cube.tag = "Walkable Cell";
                break;
            case GridCellType.Unwalkable:
                material = blockedMaterial;
                cube.tag = "Unwalkable Cell";
                break;
            case GridCellType.Start:
                material = walkableMaterial;
                cube.tag = "Start Cell";
                break;
            case GridCellType.End:
                material = endNodeMaterial;
                cube.tag = "End Cell";
                break;
            default:
                Debug.LogError("This isn't supposed to run lmao what");
                return;
        }

        // Set the color based on walkability
        cube.GetComponent<Renderer>().material = material;

        // Parent to the grid GameObject for organization
        cube.transform.parent = transform;

        // Optional: Disable colliders for visualization cubes
        Destroy(cube.GetComponent<Collider>());
    }

    public Node GetNodeFromWorldPosition(Vector3 position)
    {
        int x = Mathf.Clamp(Mathf.FloorToInt((position.x - origin.x) / cellSize), 0, grid.GetLength(0) - 1);
        int y = Mathf.Clamp(Mathf.FloorToInt((position.y - origin.y) / cellSize), 0, grid.GetLength(1) - 1);
        int z = Mathf.Clamp(Mathf.FloorToInt((position.z - origin.z) / cellSize), 0, grid.GetLength(2) - 1);

        return grid[x, y, z];
    }

    public Node GetNodeAt(int x, int y, int z)
    {
        if (x >= 0 && x < grid.GetLength(0) &&
            y >= 0 && y < grid.GetLength(1) &&
            z >= 0 && z < grid.GetLength(2))
        {
            return grid[x, y, z];
        }
        return null; // Return null if out of bounds
    }

    // Update is called once per frame
    void Update()
    {

    }
}