using UnityEngine;

public class Mole : MonoBehaviour
{
    public RobotRLAgent owner;
    private bool isHit = false;
    
    void OnTriggerEnter(Collider other)
    {
        if (isHit) return;
        
        // 找到碰撞物体所属的机器人
        RobotRLAgent agent = other.transform.root.GetComponent<RobotRLAgent>();
        
        // 只有自己的主人才能抓到这个地鼠
        if (agent != null && agent == owner)
        {
            isHit = true;
            owner.OnMoleHit();
            gameObject.SetActive(false);
        }
    }
}