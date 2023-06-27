using System.Collections;
using System.IO.Ports;
using UnityEngine;

public class ArduinoSerialReceive : MonoBehaviour
{
    SerialPort data_stream = new SerialPort("COM7", 115200, Parity.None, 8, StopBits.One);
    public string receivedString;
    public GameObject test_data;
    public Rigidbody rb;
    public float sensitivity = 0.001f;

    public string[] data;

    // Start is called before the first frame update
    void Start()
    {
        data_stream.Open(); // Initiate the serial stream
    }

    // Update is called once per frame
    void Update()
    {
        receivedString = data_stream.ReadLine();

        Debug.Log(receivedString);
        data = receivedString.Split("/");
        // rb.AddForce(0, 0, float.Parse(data[0]) * sensitivity * Time.deltaTime, ForceMode.VelocityChange);
        // rb.AddForce(float.Parse(data[1]) * sensitivity * Time.deltaTime, 0, 0, ForceMode.VelocityChange);
        transform.rotation = Quaternion.Euler(-float.Parse(data[1]), float.Parse(data[0]), -float.Parse(data[2]));
    }
}
