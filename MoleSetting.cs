using UnityEngine;

public class MoleSettings : MonoBehaviour
{
    public static MoleSettings Instance;
    
    [Header("地鼠生成设置")]
    public float spawnDistMin = 0.3f;
    public float spawnDistMax = 1.5f;
    public float spawnSideRange = 1.5f;
    
    void Awake()
    {
        Instance = this;
    }
}