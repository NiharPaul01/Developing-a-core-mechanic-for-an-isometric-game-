using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RaycastHandler : MonoBehaviour
{
    public Text positionText; // Reference to the UI Text component

    void Update()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition); // Create a ray from the mouse position
        RaycastHit hit; // Variable to store information about the raycast hit

        if (Physics.Raycast(ray, out hit)) // Perform raycast and check if it hits something
        {
            CubeInfo cubeInfo = hit.collider.GetComponent<CubeInfo>(); // Try to get CubeInfo component from the hit object

            if (cubeInfo != null) // If CubeInfo component is found
            {
                int x = cubeInfo.GetX(); // Get X coordinate from CubeInfo
                int z = cubeInfo.GetZ(); // Get Z coordinate from CubeInfo

                // Update the UI text with the grid position
                positionText.text = $"Position: ({x}, {z})";
            }
        }
    }
}
