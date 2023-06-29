using System.Collections;
using System.IO.Ports;
using UnityEngine;

public class ArduinoSerialReceive : MonoBehaviour
{
    SerialPort data_stream = new SerialPort("COM17", 115200, Parity.None, 8, StopBits.One);

    public GameObject test_data;
    public Rigidbody rb;
    public Renderer poseColor;

    public string receivedString;
    public float sensitivity = 0.001f;

    public string[] data;

    // Start is called before the first frame update
    void Start()
    {
        data_stream.Open(); // Initiate the serial stream
        rb = GetComponent<Rigidbody>();
        poseColor = gameObject.GetComponent<Renderer>();
    }

    // Update is called once per frame
    void Update()
    {
        receivedString = data_stream.ReadLine();

        // Debug.Log(receivedString);
        data = receivedString.Split("/");
        // float k = 1.5f;
        // rb.AddForce(0, 0, float.Parse(data[0]) * sensitivity * Time.deltaTime, ForceMode.VelocityChange);
        // rb.AddForce(float.Parse(data[1]) * sensitivity * Time.deltaTime, 0, 0, ForceMode.VelocityChange);

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
