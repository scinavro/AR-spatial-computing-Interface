using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Text;
using ArduinoBluetoothAPI;

public class BluetoothSerial : ArduinoSerialReceive
{
    BluetoothHelper bluetoothHelper;
    string deviceName;

    // Start is called before the first frame update
    void Start()
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
            Debug.Log(ex.Message);
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log(receivedString);
        try
        {
            if (receivedString == "") { }
            // Debug.Log("'receivedString' is empty.");
            else
            {
                data = receivedString.Split("/");
                ReflectData(data);
            }
        }
        catch (Exception ex)
        {
            Debug.Log(ex.Message);
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
}
