// ================================================================
// ===                 DECLARATION FOR MPU6050                  ===
// ================================================================

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

#define INTERRUPT_PIN 5 // use pin 2 on Arduino Uno & most boards

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

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===               DECLARATION FOR EMG SENSORS                ===
// ================================================================

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

// ================================================================
// ===               DECLARATION FOR CUSTOM USE                 ===
// ================================================================

#include "Accel_to_Displ.h"
#include "BluetoothSerial.h"

String device_name = "ESP32-BT-Slave";

BluetoothSerial SerialBT;

#define EMGBufSize 20
#define AvgBufSize 60
uint16_t EMGBuf[2][EMGBufSize];
float dataAvg[AvgBufSize] = {0};
float dataAvgAvg = 0;

#define VibMotorPin 33

enum HAND_POSE { REST = 0, FIST, SPREAD };
static int classBndry1 = 1000;
static int classBndry2 = -1500;

float deltaT = 0.1;
AccelToDispl myATD(deltaT);

void setup() {

// ================================================================
// ===                    SETUP FOR MPU6050                     ===
// ================================================================

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

    // ================================================================
    // ===                  SETUP FOR EMG SENSORS                   ===
    // ================================================================

    myFilter1.init(sampleRate, humFreq, true, true, true);
    myFilter2.init(sampleRate, humFreq, true, true, true);

    // ================================================================
    // ===                  SETUP FOR CUSTOM USE                    ===
    // ================================================================

    SerialBT.begin(device_name); // Bluetooth device name

    pinMode(VibMotorPin, OUTPUT);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

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

        if (EMG_DEBUG) {
            classifyHandPose(EMGBuf);
            Serial.print(dataAvgAvg);
            Serial.print("\t");
            Serial.print(envlope1);
            Serial.print("\t");
            Serial.print(envlope2);
            Serial.print("\t");
            Serial.print(envlope1 - envlope2);
            Serial.print("\t");
            Serial.print(2000); // y-axis scale 고정을 위한 constant
            Serial.print("\t");
            Serial.println(-2000);
        } else {
            int handPose = classifyHandPose(EMGBuf) == HAND_POSE::FIST ? 1 : classifyHandPose(EMGBuf) == HAND_POSE::SPREAD ? 2 : 0;
            String resultant;

            resultant = String(ypr[0] * 180 / M_PI) + "/" + String(ypr[1] * 180 / M_PI) + "/" + String(ypr[2] * 180 / M_PI) + "/" +
                        String(abs(myATD.X[1]) > 0.5 ? myATD.X[1] : 0) + "/" + String(abs(myATD.Y[1]) > 0.5 ? myATD.Y[1] : 0) + "/" +
                        String(abs(myATD.Z[1]) > 0.5 ? myATD.Z[1] : 0) + "/" + handPose;

            Serial.println(resultant);
            SerialBT.println(resultant);

            // Vibration motor operation
            if (handPose != HAND_POSE::REST)
                digitalWrite(VibMotorPin, HIGH);
            else
                digitalWrite(VibMotorPin, LOW);
        }

        delay(10);
    }
}

int classifyHandPose(uint16_t dataBuf[][EMGBufSize]) {
    HAND_POSE handPose;

    for (int i = 1; i < AvgBufSize; i++)
        dataAvg[i - 1] = dataAvg[i];

    dataAvg[AvgBufSize - 1] = 0;

    for (int i = 0; i < EMGBufSize; i++)
        dataAvg[AvgBufSize - 1] += dataBuf[0][i] - dataBuf[1][i];

    dataAvg[AvgBufSize - 1] /= EMGBufSize;

    for (int i = 0; i < AvgBufSize; i++)
        dataAvgAvg += dataAvg[i];

    dataAvgAvg /= AvgBufSize;

    if (dataAvgAvg > classBndry1)
        handPose = HAND_POSE::FIST;
    else if (dataAvgAvg < classBndry2)
        handPose = HAND_POSE::SPREAD;
    else
        handPose = HAND_POSE::REST;

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