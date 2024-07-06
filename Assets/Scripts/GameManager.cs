using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public static GameManager instance;    // Singleton instance of the GameManager
    public ObstacleData obstacleData;      // Reference to the ObstacleData ScriptableObject

    private void Awake()
    {
        // Ensure only one instance of GameManager exists
        if (instance == null)
        {
            instance = this;    // Set the instance to this object if it's the first one
        }
        else
        {
            Destroy(gameObject);    // Destroy any additional instances
        }
    }
}
