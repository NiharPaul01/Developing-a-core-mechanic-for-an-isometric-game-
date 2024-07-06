using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// This custom editor script allows editing of ObstacleData objects in the inspector
[CustomEditor(typeof(ObstacleData))]
public class ObstacleEditorTool : Editor
{
    // Flag to control visibility of the obstacle grid toggle
    private bool showGrid = true;

    public override void OnInspectorGUI()
    {
        // Get the target ObstacleData object
        ObstacleData obstacleData = (ObstacleData)target;

        GUILayout.Space(10);

        // Foldout to toggle visibility of the obstacle grid editor
        showGrid = EditorGUILayout.Foldout(showGrid, "Obstacle Grid");
        if (showGrid)
        {
            // Increase indent level for visual hierarchy
            EditorGUI.indentLevel++;

            // Loop through each tile in the obstacle grid (10x10 in this example)
            for (int y = 0; y < 10; y++)
            {
                EditorGUILayout.BeginHorizontal();  // Start a horizontal layout for each row
                for (int x = 0; x < 10; x++)
                {
                    // Get the current obstacle state for this tile
                    bool isObstacle = obstacleData.obstacleGrid[x, y];

                    // Display a toggle button for each tile with its current state
                    bool newIsObstacle = EditorGUILayout.Toggle(isObstacle, GUILayout.Width(20));

                    // If the toggle state changed, update the obstacle data
                    if (newIsObstacle != isObstacle)
                    {
                        obstacleData.ToggleObstacle(x, y);
                    }
                }
                EditorGUILayout.EndHorizontal();  // End the horizontal layout for each row
            }

            // Decrease indent level after the loop
            EditorGUI.indentLevel--;
        }

        GUILayout.Space(10);

        // Button to save changes made to the obstacle data
        if (GUILayout.Button("Save Obstacle Data"))
        {
            EditorUtility.SetDirty(obstacleData);  // Mark the obstacle data as dirty to trigger saving
            AssetDatabase.SaveAssets();           // Save all modified assets in the project
            AssetDatabase.Refresh();             // Refresh asset database to ensure changes are reflected
            UpdateObstaclesInScene();            // Update obstacles in the scene if needed (function defined below)
        }
    }

    void UpdateObstaclesInScene()
    {
        // Find the ObstacleManager object in the scene (assuming it exists)
        ObstacleManager obstacleManager = FindAnyObjectByType<ObstacleManager>();

        if (obstacleManager != null)
        {
            // Call the UpdateObstacles method on the ObstacleManager to update obstacles in the scene
            obstacleManager.UpdateObstacles();
        }
    }
}
