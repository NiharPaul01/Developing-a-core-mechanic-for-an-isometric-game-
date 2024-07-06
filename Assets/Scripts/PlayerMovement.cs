using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
    public float moveSpeed = 2f;        // Speed at which the player moves
    public ObstacleData obstacleData;   // Reference to obstacle data ScriptableObject
    private bool isMoving = false;      // Flag to track if the player is currently moving
    private Vector2Int gridSize = new Vector2Int(10, 10);   // Size of the grid (assuming 10x10)

    void Update()
    {
        // Check for mouse click to initiate movement and ensure player is not already moving
        if (Input.GetMouseButtonDown(0) && !isMoving)
        {
            // Cast a ray from the mouse position on screen into the scene
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                // Check if the object hit by the ray is a cube (using CubeInfo component)
                CubeInfo cubeInfo = hit.collider.GetComponent<CubeInfo>();
                if (cubeInfo != null)
                {
                    // Get target position from clicked cube's grid coordinates
                    Vector2Int targetPosition = new Vector2Int(cubeInfo.GetX(), cubeInfo.GetZ());

                    // Find a path from current player position to the target position
                    List<Vector3> path = FindPath(new Vector2Int((int)transform.position.x, (int)transform.position.z), targetPosition);

                    // If a valid path is found, start moving along the path
                    if (path != null && path.Count > 0)
                    {
                        StartCoroutine(MoveAlongPath(path));
                    }
                }
            }
        }
    }

    // Coroutine to move the player along the calculated path
    IEnumerator MoveAlongPath(List<Vector3> path)
    {
        isMoving = true; // Set flag indicating player is moving

        foreach (var targetPosition in path)
        {
            Vector3 startPosition = transform.position; // Starting position of the movement
            float distance = Vector3.Distance(startPosition, targetPosition); // Distance to move
            float startTime = Time.time; // Start time of the movement

            // Move towards the target position using Lerp until close enough
            while (Vector3.Distance(transform.position, targetPosition) > 0.01f)
            {
                float coveredDistance = (Time.time - startTime) * moveSpeed; // Distance covered so far
                float fraction = coveredDistance / distance; // Fraction of the total distance covered
                transform.position = Vector3.Lerp(startPosition, targetPosition, fraction); // Perform the movement
                transform.rotation = Quaternion.LookRotation(targetPosition - startPosition); // Rotate towards the movement direction
                yield return null; // Wait for the next frame
            }

            transform.position = targetPosition; // Ensure exact positioning at the target
        }

        isMoving = false; // Reset the movement flag once movement is complete
    }

    // Function to find a path using A* algorithm from start to target position
    List<Vector3> FindPath(Vector2Int start, Vector2Int target)
    {
        List<Node> openList = new List<Node>(); // List of nodes to be evaluated
        HashSet<Node> closedList = new HashSet<Node>(); // Set of nodes already evaluated
        Node[,] nodes = new Node[gridSize.x, gridSize.y]; // Array of nodes representing the grid

        // Initialize nodes with grid positions and obstacle information
        for (int x = 0; x < gridSize.x; x++)
        {
            for (int y = 0; y < gridSize.y; y++)
            {
                nodes[x, y] = new Node(new Vector2Int(x, y), obstacleData.obstacleGrid[x, y]);
            }
        }

        Node startNode = nodes[start.x, start.y]; // Starting node
        Node targetNode = nodes[target.x, target.y]; // Target node
        openList.Add(startNode); // Add starting node to the open list

        // A* algorithm loop
        while (openList.Count > 0)
        {
            Node currentNode = openList[0]; // Current node being evaluated

            // Find node with lowest FCost in the open list
            for (int i = 1; i < openList.Count; i++)
            {
                if (openList[i].FCost < currentNode.FCost || (openList[i].FCost == currentNode.FCost && openList[i].HCost < currentNode.HCost))
                {
                    currentNode = openList[i];
                }
            }

            openList.Remove(currentNode); // Remove current node from open list
            closedList.Add(currentNode); // Add current node to closed list

            // If target node is found, retrace the path and return waypoints
            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            // Evaluate neighboring nodes
            foreach (Node neighbor in GetNeighbors(nodes, currentNode))
            {
                // Skip obstacles and nodes already evaluated
                if (neighbor.IsObstacle || closedList.Contains(neighbor))
                {
                    continue;
                }

                // Calculate tentative GCost to neighbor node
                int newMovementCostToNeighbor = currentNode.GCost + GetDistance(currentNode, neighbor);

                // If neighbor node is not in open list or has a lower GCost, update values
                if (newMovementCostToNeighbor < neighbor.GCost || !openList.Contains(neighbor))
                {
                    neighbor.GCost = newMovementCostToNeighbor;
                    neighbor.HCost = GetDistance(neighbor, targetNode);
                    neighbor.Parent = currentNode;

                    // Add neighbor node to open list if not already present
                    if (!openList.Contains(neighbor))
                    {
                        openList.Add(neighbor);
                    }
                }
            }
        }

        return null; // Return null if no path found
    }

    // Function to get neighboring nodes of a given node
    List<Node> GetNeighbors(Node[,] nodes, Node node)
    {
        List<Node> neighbors = new List<Node>(); // List to store neighboring nodes

        // Offsets for 4-way movement (up, right, down, left)
        Vector2Int[] neighborOffsets = {
            new Vector2Int(0, 1),
            new Vector2Int(1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(-1, 0)
        };

        // Iterate through neighbor offsets
        foreach (Vector2Int offset in neighborOffsets)
        {
            Vector2Int neighborPosition = node.GridPosition + offset; // Calculate neighbor position

            // Check if neighbor position is within grid bounds and not an obstacle
            if (neighborPosition.x >= 0 && neighborPosition.x < gridSize.x && neighborPosition.y >= 0 && neighborPosition.y < gridSize.y)
            {
                neighbors.Add(nodes[neighborPosition.x, neighborPosition.y]); // Add valid neighbor node
            }
        }

        return neighbors; // Return list of neighboring nodes
    }

    // Function to calculate Manhattan distance between two nodes
    int GetDistance(Node nodeA, Node nodeB)
    {
        int distX = Mathf.Abs(nodeA.GridPosition.x - nodeB.GridPosition.x); // Horizontal distance
        int distY = Mathf.Abs(nodeA.GridPosition.y - nodeB.GridPosition.y); // Vertical distance
        return distX + distY; // Return total Manhattan distance
    }

    // Function to retrace path from end node to start node
    List<Vector3> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>(); // List to store nodes in the path
        Node currentNode = endNode; // Start with the end node

        // Traverse back through parent nodes to form the path
        while (currentNode != startNode)
        {
            path.Add(currentNode); // Add current node to the path
            currentNode = currentNode.Parent; // Move to the parent node
        }

        path.Reverse(); // Reverse the path to start from the beginning

        List<Vector3> waypoints = new List<Vector3>(); // List to store waypoints (positions)
        foreach (Node node in path)
        {
            waypoints.Add(new Vector3(node.GridPosition.x, 0.5f, node.GridPosition.y)); // Convert node positions to waypoints
        }

        return waypoints; // Return list of waypoints
    }

    // Nested class representing a node in the grid for pathfinding
    class Node
    {
        public Vector2Int GridPosition; // Position of the node in the grid
        public bool IsObstacle;         // Flag indicating if node is an obstacle
        public int GCost;               // Cost from start to current node
        public int HCost;               // Heuristic cost from current node to target node
        public Node Parent;             // Parent node for pathfinding

        public int FCost => GCost + HCost; // Total cost of the node (GCost + HCost)

        // Constructor to initialize node with grid position and obstacle flag
        public Node(Vector2Int gridPosition, bool isObstacle)
        {
            GridPosition = gridPosition; // Set grid position
            IsObstacle = isObstacle;     // Set obstacle flag
        }
    }
}
