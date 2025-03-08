using System.Collections.Generic;
using UnityEngine;

public class AStarPathfinding
{
    private Grid grid;

    public AStarPathfinding(Grid grid)
    {
        this.grid = grid;
    }

    public List<Node> FindPath(Vector3 startPosition, Vector3 endPosition)
    {
        Node startNode = grid.GetNodeFromWorldPosition(startPosition);
        Node endNode = grid.GetNodeFromWorldPosition(endPosition);

        List<Node> openSet = new(); // Nodes to evaluate
        HashSet<Node> closedSet = new(); // Nodes already evaluated

        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].F < currentNode.F || (openSet[i].F == currentNode.F && openSet[i].H < currentNode.H))
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == endNode)
            {
                return RetracePath(startNode, endNode);
            }

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (!neighbor.isWalkable || closedSet.Contains(neighbor))
                    continue;

                float newCostToNeighbor = currentNode.G + Vector3.Distance(currentNode.position, neighbor.position);
                if (newCostToNeighbor < neighbor.G || !openSet.Contains(neighbor))
                {
                    neighbor.G = newCostToNeighbor;
                    neighbor.H = Vector3.Distance(neighbor.position, endNode.position);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }

        return null; // Path not found
    }

    private List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    if (x == 0 && y == 0 && z == 0)
                        continue;

                    int checkX = node.x + x;
                    int checkY = node.y + y;
                    int checkZ = node.z + z;

                    Node neighbor = grid.GetNodeAt(checkX, checkY, checkZ);
                    if (neighbor != null)
                        neighbors.Add(neighbor);
                }
            }
        }

        return neighbors;
    }

    private List<Node> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }
}
