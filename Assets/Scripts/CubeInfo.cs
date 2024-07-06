using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CubeInfo : MonoBehaviour
{
    // Private variables to store the cube's position in the grid (assuming a grid-based system)
    private int x;
    private int z;

    // Public function to set the cube's position on the grid
    // Takes x and z coordinates as integers
    public void SetPosition(int x, int z)
    {
        // Assign the input values to the internal variables
        this.x = x;
        this.z = z;
    }

    // Public getter function to retrieve the cube's x position on the grid
    public int GetX()
    {
        // Return the stored x position
        return x;
    }

    // Public getter function to retrieve the cube's z position on the grid
    public int GetZ()
    {
        // Return the stored z position
        return z;
    }
}
