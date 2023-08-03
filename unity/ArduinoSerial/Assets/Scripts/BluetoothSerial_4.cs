using System.Collections.Generic;
using UnityEngine;
using System;
using ArduinoBluetoothAPI;

public class BluetoothSerial_4 : ArduinoSerialReceive
{
    BluetoothHelper bluetoothHelper;
    private string deviceName;

    public ModelClient client;

    private const int WINDOW_SIZE = 200;
    private const int EMG_CHANNELS = 4;

    private Queue<int> EMG1 = new Queue<int>();
    private Queue<int> EMG2 = new Queue<int>();
    private Queue<int> EMG3 = new Queue<int>();
    private Queue<int> EMG4 = new Queue<int>();

    // Start is called before the first frame update
    private void Start()
    {
        try
        {
            deviceName = "ESP32-BT-Slave";
            bluetoothHelper = BluetoothHelper.GetInstance(deviceName);

            bluetoothHelper.OnConnected += OnConnected;
            bluetoothHelper.OnConnectionFailed += OnConnectionFailed;
            bluetoothHelper.OnDataReceived += OnMessageReceived; //read the data
            bluetoothHelper.setTerminatorBasedStream("\n");

            LinkedList<BluetoothDevice> ds = bluetoothHelper.getPairedDevicesList();

            rb = GetComponent<Rigidbody>();
            poseColor = gameObject.GetComponent<Renderer>();
        }
        catch (Exception ex)
        {
            Debug.Log(ex);
        }
    }

    // Update is called once per frame
    void Update()
    {
        try
        {
            if (receivedString != "")
            {
                var data = receivedString.Split("/");
                AngleData = data[0..3];
                RotateObject(AngleData);

                EMG1.Enqueue(int.Parse(data[3]));
                EMG2.Enqueue(int.Parse(data[4]));
                EMG3.Enqueue(int.Parse(data[5]));
                EMG4.Enqueue(int.Parse(data[6]));

                if (EMG1.Count > WINDOW_SIZE)
                {
                    EMG1.Dequeue();
                    EMG2.Dequeue();
                    EMG3.Dequeue();
                    EMG4.Dequeue();
                    Predict(EMG1, EMG2, EMG3, EMG4);
                    ColorObject();
                    EMG1.Clear();
                    EMG2.Clear();
                    EMG3.Clear();
                    EMG4.Clear();
                }

                // Predict(EMG1, EMG2);
                // ColorObject();
            }
        }
        catch (Exception ex)
        {
            Debug.Log(ex);
        }

    }

    public void Predict(Queue<int> EMG1, Queue<int> EMG2, Queue<int> EMG3, Queue<int> EMG4)
    {
        int[] input = new int[WINDOW_SIZE * EMG_CHANNELS];
        EMG1.ToArray().CopyTo(input, 0);
        EMG2.ToArray().CopyTo(input, EMG1.Count);
        EMG3.ToArray().CopyTo(input, EMG1.Count * 2);
        EMG4.ToArray().CopyTo(input, EMG1.Count * 3);

        client.Predict(input, output => { pose = output; }, error => { });
    }

    void OnMessageReceived(BluetoothHelper helper)
    {
        receivedString = helper.Read();
    }

    void OnConnected(BluetoothHelper helper)
    {
        try
        {
            helper.StartListening();
            Debug.Log("Connection Successed");
        }
        catch (Exception ex)
        {
            Debug.Log(ex.Message);
        }
    }

    void OnConnectionFailed(BluetoothHelper helper)
    {
        Debug.Log("Connection Failed");
    }

    void OnGUI()
    {
        if (bluetoothHelper != null)
            bluetoothHelper.DrawGUI();
        else
            return;

        if (!bluetoothHelper.isConnected())
            if (GUI.Button(new Rect(Screen.width / 2 - Screen.width / 10, Screen.height / 10, Screen.width / 5, Screen.height / 10), "Connect"))
            {
                Debug.Log("Connecting...");
                if (bluetoothHelper.isDevicePaired())
                {
                    client = this.gameObject.AddComponent<ModelClient>();
                    bluetoothHelper.Connect(); // tries to connect
                }
                else
                    Debug.Log("Devide is not paired.");
            }

        if (bluetoothHelper.isConnected())
            if (GUI.Button(new Rect(Screen.width / 2 - Screen.width / 10, Screen.height / 10, Screen.width / 5, Screen.height / 10), "Disconnect"))
            {
                bluetoothHelper.Disconnect();
                Destroy(client);
                receivedString = "";
            }
    }
}
