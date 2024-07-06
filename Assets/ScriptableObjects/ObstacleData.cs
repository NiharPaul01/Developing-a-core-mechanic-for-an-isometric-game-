using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// This class represents data for a grid of obstacles in a Unity project
[CreateAssetMenu(fileName = "ObstacleData", menuName = "ScriptableObjects/ObstacleData", order = 1)]
public class ObstacleData : ScriptableObject
{
    // A 2D boolean array to represent the obstacle grid
    // True indicates an obstacle at that position, False indicates an empty space
    public bool[,] obstacleGrid = new bool[10, 10]; // Assuming a 10x10 grid

    // Toggles the obstacle state at a specific grid position
    // Takes x and y coordinates as integers
    public void ToggleObstacle(int x, int y)
    {
        // Invert the value at the specified grid position
        obstacleGrid[x, y] = !obstacleGrid[x, y];
    }
}

