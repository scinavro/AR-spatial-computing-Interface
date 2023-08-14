using UnityEngine;
using System;
using System.IO.Ports;

public class CableSerial : ArduinoSerialReceive
{
    SerialPort data_stream;

    // Start is called before the first frame update
    void Start()
    {
        try
        {
            data_stream = new SerialPort("COM12", 115200);
            data_stream.Open(); // Initiate the serial stream
            data_stream.ReadTimeout = 100;
            data_stream.DtrEnable = true;

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
        try
        {
            receivedString = data_stream.ReadLine();
        }
        catch (Exception ex)
        {
            Debug.Log(ex);
        }

        if (receivedString != "")
        {
            Debug.Log("hello");
            Debug.Log(receivedString);
            AngleData = receivedString.Split("/");
            RotateObject(AngleData);
        }
    }
}
