using UnityEngine;
using System;
using System.IO.Ports;

public class CableSerial : ArduinoSerialReceive
{
    SerialPort data_stream;

    // Start is called before the first frame update
    public override void Start()
    {
        base.Start();
        Debug.Log("cableserial");
        try
        {
            data_stream = new SerialPort("COM12", 9600);
            data_stream.Open(); // Initiate the serial stream
            data_stream.ReadTimeout = 1000;
            data_stream.DtrEnable = true;
        }
        catch (Exception ex)
        {
            Debug.Log(ex);
        }
    }

    // Update is called once per frame
    public virtual void Update()
    {
        try
        {
            receivedString = data_stream.ReadLine();
        }
        catch (Exception ex)
        {
            Debug.Log(ex);
        }
    }
}
