using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GridManager : MonoBehaviour
{
    public GameObject cubePrefab;         // Prefab for the grid cubes
    public GameObject obstaclePrefab;     // Prefab for obstacles
    public ObstacleData obstacleData;     // Reference to obstacle data scriptable object
    private int gridSize = 10;            // Size of the grid (assuming it's a square grid)

    void Start()
    {
        CreateGrid();       // Method to create the grid of cubes
        GenerateObstacles();    // Method to generate obstacles based on obstacleData
    }

    void CreateGrid()
    {
        // Loop through the grid size to instantiate cubes
        for (int x = 0; x < gridSize; x++)
        {
            for (int z = 0; z < gridSize; z++)
            {
                Vector3 position = new Vector3(x, 0, z);   // Calculate position for each cube
                GameObject newCube = Instantiate(cubePrefab, position, Quaternion.identity); // Instantiate cube prefab
                newCube.GetComponent<CubeInfo>().SetPosition(x, z); // Set position information using CubeInfo component
            }
        }
    }

    void GenerateObstacles()
    {
        // Loop through the grid size to generate obstacles based on obstacleData
        for (int x = 0; x < gridSize; x++)
        {
            for (int z = 0; z < gridSize; z++)
            {
                if (obstacleData.obstacleGrid[x, z])
                {
                    Vector3 position = new Vector3(x, 0.5f, z); // Adjust height as needed
                    Instantiate(obstaclePrefab, position, Quaternion.identity); // Instantiate obstacle prefab at calculated position
                }
            }
        }
    }
}
