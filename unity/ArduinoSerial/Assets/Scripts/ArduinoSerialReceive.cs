using System.Collections;
using System.IO.Ports;
using UnityEngine;

public class ArduinoSerialReceive : MonoBehaviour
{
    SerialPort data_stream = new SerialPort("COM7", 38400, Parity.None, 8, StopBits.One);
    public string receivedString;
    public GameObject test_data;
    public Rigidbody rb;
    public float sensitivity = 0.001f;

    public string[] data;

    // Start is called before the first frame update
    void Start()
    {
        data_stream.Open(); // Initiate the serial stream
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        receivedString = data_stream.ReadLine();

        // Debug.Log(receivedString);
        data = receivedString.Split("/");
        float k = 1.5f;
        // rb.AddForce(0, 0, float.Parse(data[0]) * sensitivity * Time.deltaTime, ForceMode.VelocityChange);
        // rb.AddForce(float.Parse(data[1]) * sensitivity * Time.deltaTime, 0, 0, ForceMode.VelocityChange);

        // transform.rotation = Quaternion.Euler(-float.Parse(data[1]), float.Parse(data[0]), -float.Parse(data[2]));
        rb.velocity = new Vector3(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k);
        // rb.AddForce(-float.Parse(data[4]) * k, float.Parse(data[5]) * k, float.Parse(data[3]) * k, ForceMode.VelocityChange);
    }
}
