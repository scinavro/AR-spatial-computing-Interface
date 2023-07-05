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
    SerialPort data_stream = new SerialPort("COM20", 115200, Parity.None, 8, StopBits.One);
    BluetoothHelper bluetoothHelper;
    string deviceName;

    public GameObject test_data;
    public Rigidbody rb;
    public Renderer poseColor;

    public string receivedString;
    public string[] data;

    public void ReflectData(string[] data)
    {
        transform.rotation = Quaternion.Euler(-float.Parse(data[1]), float.Parse(data[0]), -float.Parse(data[2]));
        // rb.velocity = new Vector3(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k);
        // rb.AddForce(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k, ForceMode.VelocityChange);

        int pose = int.Parse(data[6]);
        if (pose == 1) // FIST
            poseColor.material.color = Color.magenta;
        else if (pose == 2) // SPREAD
            poseColor.material.color = Color.cyan;
        else // REST
            poseColor.material.color = Color.yellow;
    }
}
