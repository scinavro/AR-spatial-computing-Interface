using UnityEngine;

public class ArduinoSerialReceive : MonoBehaviour
{
    public Rigidbody rb;
    public Renderer poseColor;

    public string receivedString;
    public string[] AngleData;
    public int pose;

    public void RotateObject(string[] AngleData)
    {
        transform.rotation = Quaternion.Euler(-float.Parse(AngleData[1]), float.Parse(AngleData[2]), -float.Parse(AngleData[0]));
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

