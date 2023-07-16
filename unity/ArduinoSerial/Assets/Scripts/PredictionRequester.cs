using System;
using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;

public class PredictionRequester : RunAbleThread
{
    private RequestSocket client;
    private Action<float[]> onOutputReceived;
    private Action<Exception> onFail;

    protected override void Run()
    {
        ForceDotNet.Force();
        using (RequestSocket client = new RequestSocket())
        {
            this.client = client;
            client.Connect("tcp://localhost:5555");

            while (Running)
            {
                byte[] outputBytes = new byte[0];
                bool gotMessage = false;
                while (Running)
                {
                    try
                    {
                        gotMessage = client.TryReceiveFrameBytes(out outputBytes);
                        if (gotMessage) break;
                    }
                    catch (Exception e) { Debug.Log(e); }
                }

                if (gotMessage)
                {
                    var output = new float[outputBytes.Length / 4];
                    Buffer.BlockCopy(outputBytes, 0, output, 0, outputBytes.Length);
                    onOutputReceived?.Invoke(output);
                }
            }

        }

        NetMQConfig.Cleanup();
    }

    public void SendInput(float[] input)
    {
        try
        {
            var byteArray = new byte[input.Length * 4];
            Buffer.BlockCopy(input, 0, byteArray, 0, byteArray.Length);
            client.SendFrame(byteArray);
        }
        catch (Exception e)
        {
            onFail(e);
        }
    }

    public void SetOnTextReceivedListener(Action<float[]> onOutputReceived, Action<Exception> fallback)
    {
        this.onOutputReceived = onOutputReceived;
        onFail = fallback;
    }
}