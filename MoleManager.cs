using UnityEngine;

public class MoleManager : MonoBehaviour
{
    public GameObject molePrefab;    // 地鼠预制体
    public float spawnRange = 2f;    // 生成范围
    public Transform robotTransform; // 机器人位置
    
    private GameObject currentMole;
    private int score = 0;
    
    void Start()
    {
        SpawnMole();
    }
    
    // 生成新地鼠
    public void SpawnMole()
    {
        if (currentMole != null)
        {
            Destroy(currentMole);
        }
        
        // 在机器人周围随机位置生成
        Vector3 randomPos = new Vector3(
            robotTransform.position.x + Random.Range(-spawnRange, spawnRange),
            -0.45f,  // 贴近地面
            robotTransform.position.z + Random.Range(-spawnRange, spawnRange)
        );
        
        currentMole = Instantiate(molePrefab, randomPos, Quaternion.identity);
        currentMole.tag = "Mole";
    }
    
    // 获取当前地鼠
    public GameObject GetCurrentMole()
    {
        return currentMole;
    }
    
    // 地鼠被消灭
    public void MoleHit()
    {
        score++;
        Debug.Log("得分: " + score);
        SpawnMole();
    }
    
    // 获取分数
    public int GetScore()
    {
        return score;
    }
    
    // 重置游戏
    public void ResetGame()
    {
        score = 0;
        SpawnMole();
    }
}