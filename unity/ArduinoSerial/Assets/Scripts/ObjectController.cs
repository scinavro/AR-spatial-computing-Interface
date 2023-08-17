using UnityEngine;

public class ObjectController : CableSerial
{


    public override void Start()
    {
        base.Start();
        Debug.Log("rotatearm");
    }

    public override void Update()
    {
        base.Update();

    }


    public void FixedUpdate()
    {
        Debug.Log(receivedString);
        AngleData = receivedString.Split("/")[0..3];
        AccelData = receivedString.Split("/")[3..6];
        RotateObject(AngleData);
        MoveObject(AccelData);
    }
}