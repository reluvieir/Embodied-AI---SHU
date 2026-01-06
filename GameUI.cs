using UnityEngine;
using UnityEngine.UI;

public class GameUI : MonoBehaviour
{
    public static GameUI Instance;
    
    public Text scoreText;
    public Text timerText;
    public Text catchCountText;
    
    private int totalScore = 0;
    private int catchCount = 0;
    private float gameTime = 0f;
    
    void Awake()
    {
        Instance = this;
    }
    
    void Update()
    {
        // 更新计时器
        gameTime += Time.deltaTime;
        if (timerText != null)
        {
            int minutes = (int)(gameTime / 60);
            int seconds = (int)(gameTime % 60);
            timerText.text = string.Format("时间: {0:00}:{1:00}", minutes, seconds);
        }
    }
    
    // 抓到地鼠时调用
    public void AddScore(int points)
    {
        totalScore += points;
        catchCount++;
        
        if (scoreText != null)
            scoreText.text = "分数: " + totalScore;
        
        if (catchCountText != null)
            catchCountText.text = "抓到: " + catchCount + " 只";
    }
    
    // 重置
    public void ResetGame()
    {
        totalScore = 0;
        catchCount = 0;
        gameTime = 0f;
    }
}