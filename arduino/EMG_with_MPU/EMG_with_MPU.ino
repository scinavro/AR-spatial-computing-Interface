// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// using DMP (MotionApps v2.0) 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
// https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking
// code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility
//      error 2012-06-20 - improved FIFO overflow handling and simplified read
//      process 2012-06-19 - completely rearranged DMP initialization code and
//      simplification 2012-06-13 - pull gyro and accel data from FIFO packet
//      instead of reading directly 2012-06-09 - fix broken FIFO read sequence
//      and change interrupt detection to RISING 2012-06-05 - add
//      gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks
//      Sungon Lee) 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec
//      instead of 250 2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation
// board) AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

#define INTERRUPT_PIN 5 // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success,
                        // !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

// ###################################################################################
// ###################################################################################
// ###################################################################################
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

#define EMG_DEBUG 0

#define SensorInputPin1 36 // input pin number
#define SensorInputPin2 39

EMGFilters myFilter1;
EMGFilters myFilter2;
// discrete filters must works with fixed sample frequence
// our emg filter only support "SAMPLE_FREQ_500HZ" or "SAMPLE_FREQ_1000HZ"
// other sampleRate inputs will bypass all the EMG_FILTER
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
// For countries where power transmission is at 50 Hz
// For countries where power transmission is at 60 Hz, need to change to
// "NOTCH_FREQ_60HZ"
// our emg filter only support 50Hz and 60Hz input
// other inputs will bypass all the EMG_FILTER
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_60HZ;

// Calibration:
// put on the sensors, and release your muscles;
// wait a few seconds, and select the max value as the Threshold;
// any value under Threshold will be set to zero
static int Threshold1 = 0;
static int Threshold2 = 0;

unsigned long timeStamp;
unsigned long timeBudget;
// ###################################################################################
// ###################################################################################
// ###################################################################################
#include "Accel_to_Displ.h"
#include "BluetoothSerial.h"

String device_name = "ESP32-BT-Slave";

BluetoothSerial SerialBT;

#define EMGBufSize 20
uint16_t EMGBuf[2][EMGBufSize];

#define VibMotorPin 33

enum HAND_POSE { REST = 0, WAVE_IN, WAVE_OUT };
static int classBndry1 = 5000;
static int classBndry2 = 10000;

float deltaT = 0.1;
AccelToDispl myATD(deltaT);

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
                           // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo:
    // ")); while (Serial.available() && Serial.read()); // empty buffer while
    // (!Serial.available());                 // wait for data while
    // (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to
        // use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // ###################################################################################
    // ###################################################################################
    // ###################################################################################
    myFilter1.init(sampleRate, humFreq, true, true, true);
    myFilter2.init(sampleRate, humFreq, true, true, true);
    // setup for time cost measure
    // using micros()
    // timeBudget = 1e6 / sampleRate;
    // micros will overflow and auto return to zero every 70 minutes
    // ###################################################################################
    // ###################################################################################
    // ###################################################################################
    SerialBT.begin(device_name); // Bluetooth device name

    pinMode(VibMotorPin, OUTPUT);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
                                                   // display quaternion values in easy matrix form: w x
                                                   // y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.print(q.w);
        Serial.print("/");
        Serial.print(q.x);
        Serial.print("/");
        Serial.print(q.y);
        Serial.print("/");
        Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);

        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("/");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("/");
        Serial.println(euler[2] * 180 / M_PI);
        delay(10);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // ###################################################################################
        // ###################################################################################
        // ###################################################################################
        /* add main program code here */
        // In order to make sure the ADC sample frequence on arduino,
        // the time cost should be measured each loop
        /*------------start here-------------------*/
        // timeStamp = micros();

        int Value1 = analogRead(SensorInputPin1);
        int Value2 = analogRead(SensorInputPin2);

        // filter processing
        int DataAfterFilter1 = myFilter1.update(Value1);
        int DataAfterFilter2 = myFilter2.update(Value2);

        int envlope1 = sq(DataAfterFilter1);
        int envlope2 = sq(DataAfterFilter2);
        // any value under Threshold will be set to zero
        envlope1 = (envlope1 > Threshold1) ? envlope1 : 0;
        envlope2 = (envlope2 > Threshold2) ? envlope2 : 0;

        pushEMGBuf(envlope1, envlope2, EMGBuf);

        int handPose = classifyHandPose(EMGBuf) == HAND_POSE::WAVE_IN ? 1 : classifyHandPose(EMGBuf) == HAND_POSE::WAVE_OUT ? 2 : 0;

        /*------------end here---------------------*/
        // if less than timeBudget, then you still have (timeBudget - timeStamp)
        // to do your work delayMicroseconds(500); if more than timeBudget, the
        // sample rate need to reduce to SAMPLE_FREQ_500HZ
        // ###################################################################################
        // ###################################################################################
        // ###################################################################################

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        myATD.setAccelData(aaWorld.x, aaWorld.y, aaWorld.z);
        myATD.filter(0);
        myATD.integral(1);
        myATD.filter(1);
        // myATD.integral(2);
        // myATD.filter(2);

        if (EMG_DEBUG) {
            Serial.print(envlope1);
            Serial.print("\t");
            Serial.print(envlope2);
            Serial.print("\t");
            Serial.println(100000); // y-axis scale 고정을 위한 constant
            // Serial.print("Filters cost time: ");
            // Serial.println(timeStamp);
            // the filter cost average around 520 us
        } else {
            String resultant;
            resultant = String(ypr[0] * 180 / M_PI) + "/" + String(ypr[1] * 180 / M_PI) + "/" + String(ypr[2] * 180 / M_PI) + "/" +
                        String(abs(myATD.X[1]) > 0.5 ? myATD.X[1] : 0) + "/" + String(abs(myATD.Y[1]) > 0.5 ? myATD.Y[1] : 0) + "/" +
                        String(abs(myATD.Z[1]) > 0.5 ? myATD.Z[1] : 0) + "/" + handPose;

            Serial.println(resultant);
            SerialBT.println(resultant);
        }

        if (handPose != HAND_POSE::REST)
            digitalWrite(VibMotorPin, HIGH);
        else
            digitalWrite(VibMotorPin, LOW);

        delay(10);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        velX += aaReal.x * deltaT;
        velY += aaReal.y * deltaT;
        velZ += aaReal.z * deltaT;
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.print(aaReal.z);
        Serial.print("\t");
        Serial.print(velX);
        Serial.print("\t");
        Serial.print(velY);
        Serial.print("\t");
        Serial.println(velZ);
        delay(10);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        myATD.setAccelData(aaWorld.x, aaWorld.y, aaWorld.z);
        myATD.filter(0);
        myATD.integral(1);
        // myATD.filter(1);
        // myATD.integral(2);
        // myATD.filter(2);

        int Value1 = analogRead(SensorInputPin1);
        int Value2 = analogRead(SensorInputPin2);

        // filter processing
        int DataAfterFilter1 = myFilter1.update(Value1);
        int DataAfterFilter2 = myFilter2.update(Value2);

        int envlope1 = sq(DataAfterFilter1);
        int envlope2 = sq(DataAfterFilter2);
        // any value under Threshold will be set to zero
        envlope1 = (envlope1 > Threshold1) ? envlope1 : 0;
        envlope2 = (envlope2 > Threshold2) ? envlope2 : 0;

        pushBuf(envlope1, envlope2, EMGBuf);

        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("/");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("/");
        Serial.print(ypr[2] * 180 / M_PI);
        Serial.print("/");
        Serial.print(myATD.X[1]);
        Serial.print("/");
        Serial.print(myATD.Y[1]);
        Serial.print("/");
        Serial.print(myATD.Z[1]);
        Serial.print("/");
        Serial.println(classifyHandPose(EMGBuf) == HAND_POSE::REST ? 0 : classifyHandPose(EMGBuf) == HAND_POSE::WAVE_IN ? 1 : 2);
        delay(10);
#endif

#ifdef OUTPUT_TEAPOT
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
    }
}

int classifyHandPose(uint16_t dataBuf[][EMGBufSize]) {
    HAND_POSE handPose;
    int avgData1 = 0;
    int avgData2 = 0;

    for (int i = 0; i < EMGBufSize; i++) {
        avgData1 += dataBuf[0][i];
        avgData2 += dataBuf[1][i];
    }
    avgData1 /= EMGBufSize;
    avgData2 /= EMGBufSize;

    if (avgData1 > classBndry1 && avgData2 <= classBndry2) {
        handPose = HAND_POSE::WAVE_IN;
    } else if (avgData1 <= classBndry1 && avgData2 > classBndry2) {
        handPose = HAND_POSE::WAVE_OUT;
    } else {
        handPose = HAND_POSE::REST;
    }

    return handPose;
}

void pushEMGBuf(int data1, int data2, uint16_t dataBuf[][EMGBufSize]) {
    for (int i = 1; i < EMGBufSize; i++) {
        dataBuf[0][i - 1] = dataBuf[0][i];
        dataBuf[1][i - 1] = dataBuf[1][i];
    }
    dataBuf[0][EMGBufSize - 1] = data1;
    dataBuf[1][EMGBufSize - 1] = data2;
}