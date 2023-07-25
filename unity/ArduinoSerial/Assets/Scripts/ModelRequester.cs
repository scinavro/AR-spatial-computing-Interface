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
    private Action<int> onOutputReceived;
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
                byte[] outputBytes = new byte[4];
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
                    var _temp = BitConverter.ToString(outputBytes).ToLower();
                    var temp = _temp.Split("-");
                    byte[] a = new byte[4];
                    for (int i = 0; i < 4; i++)
                    {
                        a[i] = Convert.ToByte(temp[i], 16);
                    }
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(a);
                    int output = BitConverter.ToInt32(a, 0);

                    Debug.Log("Received: " + output);
                    onOutputReceived?.Invoke(output);
                }
            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }

    public void SendInput(int[] input)
    {
        try
        {
            Debug.Log("Sending Data...");
            var EMGArray = new byte[input.Length * 4];
            Buffer.BlockCopy(input, 0, EMGArray, 0, EMGArray.Length);

            client.SendFrame(EMGArray);
        }
        catch (Exception e)
        {
            Debug.Log(e);
        }
    }

    public void SetOnTextReceivedListener(Action<int> onOutputReceived, Action<Exception> fallback)
    {
        this.onOutputReceived = onOutputReceived;
        onFail = fallback;
    }
}