/*
    Arduino and MPU6050 IMU - 3D Visualization Example
 by Dejan, https://howtomechatronics.com
 */
import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
Serial myPort;
String data="";
float disX, disY, disZ;
float yaw, pitch, roll;
int pose;
void setup() {
  size (1560, 720, P3D);
  myPort = new Serial(this, "COM7", 38400); // starts the serial communication
  myPort.bufferUntil('\n');
}
void draw() {
  translate(width/2, height/2, 0);
  translate(-disY, -disZ, -disX);
  background(233);
  textSize(22);
  text("X: " + disX + "    Y: " + disY + "    Z: " + disZ, -100, 265);
  text("Yaw: " + int(yaw) + "    Pitch: " + int(pitch) + "    Roll: " + int(roll), -100, 300);
  //rotateY(radians(yaw));
  //rotateX(radians(-pitch));
  //rotateZ(radians(roll));
  

  // 3D 0bject
  textSize(30);
  if (pose == 0) {  // REST
    fill(0, 255, 128);
  } else if (pose == 1) {  // WAVE IN
    fill(255, 102, 102);
  } else if (pose == 2) {  // WAVE OUT
    fill(178, 102, 255);
  }
  box (250, 40, 550); // Draw box
  textSize(25);
  fill(255, 255, 255);
  text("AR Spatial Interface", -100, 10, 276);
  delay(10);
  //println("ypr:\t" + angleX + "\t" + angleY); // Print the values to check whether we are getting proper values
}
// Read data from the Serial Port
void serialEvent (Serial myPort) {
  try {
    // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
    data = myPort.readStringUntil('\n');
    // if you got any bytes other than the linefeed:
    if (data != null) {
      data = trim(data);
      // split the string at "/"
      String items[] = split(data, '/');
      if (items.length > 1) {
        //--- Roll,Pitch in degrees
        yaw = -float(items[0]);
        pitch = float(items[1]);
        roll = float(items[2]);
        disX = float(items[3]);
        disY = float(items[4]);
        disZ = float(items[5]);
        pose = int(items[6]);
      }
    }
  }
  catch(RuntimeException e) {
    e.printStackTrace();
  }
}
