using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Text;
using System.IO.Ports;

public class CableSerial : ArduinoSerialReceive
{
    SerialPort data_stream = new SerialPort("COM17", 115200, Parity.None, 8, StopBits.One);

    // Start is called before the first frame update
    void Start()
    {
        try
        {
            data_stream.Open(); // Initiate the serial stream

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
            receivedString = data_stream.ReadLine();

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
}
