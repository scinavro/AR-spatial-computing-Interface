using System;
using UnityEngine;

public class ModelClient : MonoBehaviour
{
    private ModelRequester _modelRequester;

    public void Start() => InitializeServer();

    public void InitializeServer()
    {
        Debug.Log("requester start");
        _modelRequester = new ModelRequester();
        _modelRequester.Start();
    }

    public void Predict(int[] input, Action<int> onOutputReceived, Action<Exception> fallback)
    {
        _modelRequester.SetOnTextReceivedListener(onOutputReceived, fallback);
        _modelRequester.SendInput(input);
    }

    public void OnDestroy()
    {
        _modelRequester.Stop();
    }
}