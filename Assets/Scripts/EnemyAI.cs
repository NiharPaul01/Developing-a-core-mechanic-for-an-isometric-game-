using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnemyAI : MonoBehaviour
{
    public float moveSpeed = 2f;                // Speed at which the enemy moves
    public ObstacleData obstacleData;           // Reference to obstacle data scriptable object
    public Transform playerTransform;           // Reference to the player's transform
    private Vector2Int gridSize = new Vector2Int(10, 10);  // Grid size for pathfinding

    void Start()
    {
        StartCoroutine(MoveTowardsPlayer());   // Start moving towards the player
    }

    IEnumerator MoveTowardsPlayer()
    {
        while (true)
        {
            // Determine player's position on the grid
            Vector2Int playerPosition = new Vector2Int((int)playerTransform.position.x, (int)playerTransform.position.z);
            // Find the nearest adjacent position to the player
            Vector2Int nearestPosition = FindNearestAdjacentPosition(playerPosition);

            if (nearestPosition != Vector2Int.zero)
            {
                // Find path from current position to the nearest adjacent position
                List<Vector3> path = FindPath(new Vector2Int((int)transform.position.x, (int)transform.position.z), nearestPosition);

                if (path != null && path.Count > 0)
                {
                    // Move along the computed path
                    yield return StartCoroutine(MoveAlongPath(path));
                }
            }

            yield return new WaitForSeconds(1f); // Adjust delay as needed
        }
    }

    Vector2Int FindNearestAdjacentPosition(Vector2Int playerPosition)
    {
        // Get all adjacent positions to the player's position
        Vector2Int[] adjacentPositions = GetAdjacentPositions(playerPosition);
        Vector2Int nearestPosition = Vector2Int.zero;
        float minDistance = float.MaxValue;

        // Find the closest unoccupied adjacent position
        foreach (Vector2Int adjPos in adjacentPositions)
        {
            if (!IsOccupied(adjPos))
            {
                float distance = Vector2Int.Distance(adjPos, playerPosition);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestPosition = adjPos;
                }
            }
        }

        return nearestPosition;
    }

    IEnumerator MoveAlongPath(List<Vector3> path)
    {
        // Move the enemy along the calculated path
        foreach (var targetPosition in path)
        {
            Vector3 startPosition = transform.position;
            float distance = Vector3.Distance(startPosition, targetPosition);
            float startTime = Time.time;

            // Smoothly move towards the target position
            while (Vector3.Distance(transform.position, targetPosition) > 0.01f)
            {
                float coveredDistance = (Time.time - startTime) * moveSpeed;
                float fraction = coveredDistance / distance;
                transform.position = Vector3.Lerp(startPosition, targetPosition, fraction);

                // Rotate towards the movement direction
                transform.rotation = Quaternion.LookRotation(targetPosition - transform.position);

                yield return null;
            }

            transform.position = targetPosition;
        }
    }

    List<Vector3> FindPath(Vector2Int start, Vector2Int target)
    {
        // A* pathfinding algorithm to find a path from start to target position
        List<Node> openList = new List<Node>();
        HashSet<Node> closedList = new HashSet<Node>();
        Node[,] nodes = new Node[gridSize.x, gridSize.y];

        // Initialize nodes grid
        for (int x = 0; x < gridSize.x; x++)
        {
            for (int y = 0; y < gridSize.y; y++)
            {
                nodes[x, y] = new Node(new Vector2Int(x, y), obstacleData.obstacleGrid[x, y]);
            }
        }

        Node startNode = nodes[start.x, start.y];
        Node targetNode = nodes[target.x, target.y];
        openList.Add(startNode);

        // Search for the path
        while (openList.Count > 0)
        {
            Node currentNode = openList[0];

            // Find node with lowest FCost in openList
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].FCost < currentNode.FCost || (openList[i].FCost == currentNode.FCost && openList[i].HCost < currentNode.HCost))
                {
                    currentNode = openList[i];
                }
            }

            openList.Remove(currentNode);
            closedList.Add(currentNode);

            // Path found
            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            // Explore neighbors
            foreach (Node neighbor in GetNeighbors(nodes, currentNode))
            {
                if (neighbor.IsObstacle || closedList.Contains(neighbor))
                {
                    continue;
                }

                int newMovementCostToNeighbor = currentNode.GCost + GetDistance(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.GCost || !openList.Contains(neighbor))
                {
                    neighbor.GCost = newMovementCostToNeighbor;
                    neighbor.HCost = GetDistance(neighbor, targetNode);
                    neighbor.Parent = currentNode;

                    if (!openList.Contains(neighbor))
                    {
                        openList.Add(neighbor);
                    }
                }
            }
        }

        return null; // Path not found
    }

    List<Node> GetNeighbors(Node[,] nodes, Node node)
    {
        // Get valid neighbors for the current node
        List<Node> neighbors = new List<Node>();

        Vector2Int[] neighborOffsets = {
            new Vector2Int(0, 1),
            new Vector2Int(1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(-1, 0)
        };

        foreach (Vector2Int offset in neighborOffsets)
        {
            Vector2Int neighborPosition = node.GridPosition + offset;
            if (IsWithinBounds(neighborPosition) && !nodes[neighborPosition.x, neighborPosition.y].IsObstacle)
            {
                neighbors.Add(nodes[neighborPosition.x, neighborPosition.y]);
            }
        }

        return neighbors;
    }

    bool IsWithinBounds(Vector2Int position)
    {
        // Check if a position is within the grid bounds
        return position.x >= 0 && position.x < gridSize.x && position.y >= 0 && position.y < gridSize.y;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        // Calculate distance between two nodes (Manhattan distance)
        int distX = Mathf.Abs(nodeA.GridPosition.x - nodeB.GridPosition.x);
        int distY = Mathf.Abs(nodeA.GridPosition.y - nodeB.GridPosition.y);
        return distX + distY;
    }

    List<Vector3> RetracePath(Node startNode, Node endNode)
    {
        // Retrace the path from endNode to startNode
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.Parent;
        }
        path.Reverse();

        // Convert path to world space waypoints
        List<Vector3> waypoints = new List<Vector3>();
        foreach (Node node in path)
        {
            waypoints.Add(new Vector3(node.GridPosition.x, 0.5f, node.GridPosition.y));
        }
        return waypoints;
    }

    Vector2Int[] GetAdjacentPositions(Vector2Int center)
    {
        // Get adjacent positions around a center position
        Vector2Int[] adjacentPositions = {
            new Vector2Int(center.x, center.y + 1),
            new Vector2Int(center.x + 1, center.y),
            new Vector2Int(center.x, center.y - 1),
            new Vector2Int(center.x - 1, center.y)
        };
        return adjacentPositions;
    }

    bool IsOccupied(Vector2Int position)
    {
        // Check if a position is occupied by an obstacle or the player
        if (!IsWithinBounds(position))
            return true;

        return obstacleData.obstacleGrid[position.x, position.y] || ((int)playerTransform.position.x == position.x && (int)playerTransform.position.z == position.y);
    }

    // Node class representing a cell in the grid
    class Node
    {
        public Vector2Int GridPosition; // Grid position of the node
        public bool IsObstacle;         // Whether the node is an obstacle
        public int GCost;               // Cost from start to current node
        public int HCost;               // Heuristic cost from current node to target node
        public Node Parent;             // Parent node for pathfinding

        public int FCost => GCost + HCost; // Total cost of the node

        public Node(Vector2Int gridPosition, bool isObstacle)
        {
            GridPosition = gridPosition;
            IsObstacle = isObstacle;
        }
    }
}
