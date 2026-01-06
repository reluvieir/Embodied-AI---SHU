using UnityEngine;
using Unity.MLAgents;

public class MoleHunter : MonoBehaviour
{
    public MoleManager moleManager;
    
    public GameObject GetMole()
    {
        if (moleManager != null)
        {
            return moleManager.GetCurrentMole();
        }
        return null;
    }
}