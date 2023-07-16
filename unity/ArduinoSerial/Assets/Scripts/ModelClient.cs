using UnityEngine;

public class ModelClient : MonoBehaviour
{
    private ModelRequester _modelRequester;

    private void Start()
    {
        _modelRequester = new ModelRequester();
        _modelRequester.Start();
    }

    private void OnDestroy()
    {
        _modelRequester.Stop();
    }
}