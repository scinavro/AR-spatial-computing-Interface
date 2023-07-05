using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using ArduinoBluetoothAPI;
using System.IO.Ports;
using System;
using System.Text;

public class ArduinoSerialReceive : MonoBehaviour
{
    // SerialPort data_stream = new SerialPort("COM20", 115200, Parity.None, 8, StopBits.One);
    BluetoothHelper bluetoothHelper;
    string deviceName;

    public GameObject test_data;
    public Rigidbody rb;
    public Renderer poseColor;

    public string receivedString;
    public float sensitivity = 0.001f;

    public string[] data;

    // Start is called before the first frame update
    void Start()
    {
        deviceName = "ESP32-BT-Slave";
        try
        {
            // data_stream.Open(); // Initiate the serial stream
            bluetoothHelper = BluetoothHelper.GetInstance(deviceName);
            bluetoothHelper.OnConnected += OnConnected;
            bluetoothHelper.OnConnectionFailed += OnConnectionFailed;
            bluetoothHelper.OnDataReceived += OnMessageReceived; //read the data
            bluetoothHelper.setTerminatorBasedStream("\n");
            LinkedList<BluetoothDevice> ds = bluetoothHelper.getPairedDevicesList();
        }
        catch (Exception ex)
        {
            Debug.Log(ex.Message);
        }

        rb = GetComponent<Rigidbody>();
        poseColor = gameObject.GetComponent<Renderer>();
    }

    // Update is called once per frame
    void Update()
    {
        // receivedString = data_stream.ReadLine();
        // Debug.Log(receivedString);

        if (receivedString == "") { }
        // Debug.Log("'receivedString' is empty.");
        else
        {


            data = receivedString.Split("/");

            transform.rotation = Quaternion.Euler(-float.Parse(data[1]), float.Parse(data[0]), -float.Parse(data[2]));
            // rb.velocity = new Vector3(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k);
            // rb.AddForce(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k, ForceMode.VelocityChange);

            int pose = int.Parse(data[6]);
            if (pose == 1) // WAVE IN
                poseColor.material.color = Color.magenta;
            else if (pose == 2) // WAVE OUT
                poseColor.material.color = Color.cyan;
            else // REST
                poseColor.material.color = Color.yellow;
        }
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
                    bluetoothHelper.Connect(); // tries to connect
                else
                    Debug.Log("Devide is not paired.");
            }

        if (bluetoothHelper.isConnected())
            if (GUI.Button(new Rect(Screen.width / 2 - Screen.width / 10, Screen.height / 10, Screen.width / 5, Screen.height / 10), "Disconnect"))
            {
                bluetoothHelper.Disconnect();
            }
    }

    void OnDestroy()
    {
        if (bluetoothHelper != null)
            bluetoothHelper.Disconnect();
    }
}
