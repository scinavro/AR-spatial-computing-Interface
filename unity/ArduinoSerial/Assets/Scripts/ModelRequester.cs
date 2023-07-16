using System;
using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;

/// <summary>
///     Example of requester who only sends Hello. Very nice guy.
///     You can copy this class and modify Run() to suits your needs.
///     To use this class, you just instantiate, call Start() when you want to start and Stop() when you want to stop.
/// </summary>
public class ModelRequester : RunAbleThread
{
    private RequestSocket client;
    private Action<float[]> onOutputReceived;
    private Action<Exception> onFail;
    /// <summary>
    ///     Request Hello message to server and receive message back. Do it 10 times.
    ///     Stop requesting when Running=false.
    /// </summary>
    protected override void Run()
    {
        ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet
        using (RequestSocket client = new RequestSocket())
        {
            this.client = client;
            client.Connect("tcp://localhost:5555");

            while (Running)
            {
                Debug.Log("Sending Hello...");
                client.SendFrame("Hello");

                byte[] outputBytes = new byte[0];
                bool gotMessage = false;
                while (Running)
                {
                    try
                    {
                        Debug.Log("Receiving Bytes...");
                        gotMessage = client.TryReceiveFrameBytes(out outputBytes);
                        if (gotMessage) break;
                    }
                    catch (Exception e) { Debug.Log(e); }
                }

                if (gotMessage)
                {
                    Debug.Log("Received " + System.Text.Encoding.Default.GetString(outputBytes));

                    // var output = new float[outputBytes.Length / 4];
                    // Buffer.BlockCopy(outputBytes, 0, output, 0, outputBytes.Length);
                    // onOutputReceived?.Invoke(output);
                }
            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
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
}