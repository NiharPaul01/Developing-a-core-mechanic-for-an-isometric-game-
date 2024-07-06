using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleManager : MonoBehaviour
{
    public GameObject obstaclePrefab;   // Prefab for obstacles (assign in Inspector)
    public ObstacleData obstacleData;   // Reference to ObstacleData ScriptableObject (assign in Inspector)

    private GameObject[,] obstacleInstances = new GameObject[10, 10]; // Array to store instantiated obstacle game objects

    void Start()
    {
        GenerateObstacles();    // Method to generate initial obstacles based on obstacleData
    }

    void GenerateObstacles()
    {
        // Loop through the grid size (assuming 10x10) to generate obstacles
        for (int x = 0; x < 10; x++)
        {
            for (int y = 0; y < 10; y++)
            {
                // Check if obstacle should be placed at current grid position
                if (obstacleData.obstacleGrid[x, y])
                {
                    Vector3 position = new Vector3(x, 0.5f, y); // Adjust height as needed
                    GameObject obstacle = Instantiate(obstaclePrefab, position, Quaternion.identity); // Instantiate obstacle prefab
                    obstacleInstances[x, y] = obstacle; // Store reference to instantiated obstacle
                }
            }
        }
    }

    public void UpdateObstacles()
    {
        // Loop through the grid size (assuming 10x10) to update obstacles
        for (int x = 0; x < 10; x++)
        {
            for (int y = 0; y < 10; y++)
            {
                // If there is an existing obstacle instance, destroy it
                if (obstacleInstances[x, y] != null)
                {
                    Destroy(obstacleInstances[x, y]);
                    obstacleInstances[x, y] = null; // Clear reference to destroyed obstacle
                }

                // Check if obstacle should be placed at current grid position
                if (obstacleData.obstacleGrid[x, y])
                {
                    Vector3 position = new Vector3(x, 0.5f, y); // Adjust height as needed
                    GameObject obstacle = Instantiate(obstaclePrefab, position, Quaternion.identity); // Instantiate obstacle prefab
                    obstacleInstances[x, y] = obstacle; // Store reference to instantiated obstacle
                }
            }
        }
    }
}
