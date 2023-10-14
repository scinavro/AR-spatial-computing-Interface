using UnityEngine;

public class ArduinoSerialReceive : MonoBehaviour
{
    public GameObject Ball, Pivot;
    public Rigidbody rb_ball, rb_pivot;
    public Renderer poseColor;

    public string receivedString;
    public string[] AngleData = new string[3];
    public string[] AccelData = new string[3];
    public int pose;

    public int accelDrift1 = 500;
    public int accelDrift2 = 500;
    public int accelThreshold = 200;
    public float accelPower = 0.01f;
    public float jumpPower = 0.3f;

    public virtual void Start()
    {
        Debug.Log("arduinoserialreceive");
        Ball = GameObject.FindGameObjectWithTag("Ball");
        Pivot = GameObject.FindGameObjectWithTag("Pivot");
        rb_ball = Ball.GetComponent<Rigidbody>();
        rb_pivot = Pivot.GetComponent<Rigidbody>();

        poseColor = gameObject.GetComponent<Renderer>();
    }

    public void RotateObject(string[] AngleData)
    {
        Pivot.transform.rotation = Quaternion.Euler(-float.Parse(AngleData[1]), float.Parse(AngleData[2]), -float.Parse(AngleData[0]));
    }

    public void MoveObject(string[] AccelData)
    {
        // var xAccel = Mathf.Abs(float.Parse(AccelData[0]) - accelDrift1) > accelThreshold ? float.Parse(AccelData[0]) - accelDrift1 : 0;
        // var zAccel = Mathf.Abs(float.Parse(AccelData[1]) - accelDrift1) > accelThreshold ? -(float.Parse(AccelData[1]) - accelDrift1) : 0;
        var xAccel = Mathf.Abs(float.Parse(AccelData[1]) - accelDrift1) > accelThreshold ? float.Parse(AccelData[1]) - accelDrift1 : 0;
        var zAccel = Mathf.Abs(float.Parse(AccelData[0]) - accelDrift1) > accelThreshold ? float.Parse(AccelData[0]) - accelDrift1 : 0;
        Vector3 movement = new Vector3(xAccel, 0, zAccel);
        int jump = int.Parse(AccelData[2]);

        rb_ball.AddForce(movement * accelPower / 1000, ForceMode.Force);
        rb_ball.AddForce(Vector3.up * jump * jumpPower, ForceMode.Impulse);

    }

    public void ColorObject()
    {



        if (pose == 1) // FIST
            poseColor.material.color = Color.magenta;
        else if (pose == 2) // SPREAD
            poseColor.material.color = Color.cyan;
        else // REST
            poseColor.material.color = Color.yellow;
    }
}

