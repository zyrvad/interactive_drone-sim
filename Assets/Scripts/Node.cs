using UnityEngine;

public class Node
{
    public Vector3 position; // World position of the node
    public bool isWalkable;  // Is the node walkable (if it's not blocked)
    public int x, y, z;      // The grid coordinates (X, Y, Z) in the grid array
    public float G;      // Cost from the start node to this node
    public float H;      // Heuristic cost from this node to the end node
    public float F => G + H; // Total cost (fCost = gCost + hCost)
    public Node parent;      // The parent node to trace the path back

    // Constructor for the Node
    public Node(Vector3 position, bool isWalkable)
    {
        this.position = position;
        this.isWalkable = isWalkable;
    }

    // Set the grid coordinates for the node
    public void SetGridCoordinates(int x, int y, int z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}
