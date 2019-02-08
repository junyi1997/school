

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69); // <-- use for AD0 high

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



#define OUTPUT_READABLE_YAWPITCHROLL





#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1;           // [w, x, y, z]         quaternion container
Quaternion q2;           // [w, x, y, z]         quaternion container

VectorFloat gravity1;    // [x, y, z]            gravity vector
VectorFloat gravity2;    // [x, y, z]            gravity vector

float ypr1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    mpu2.initialize();
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus1 = mpu1.dmpInitialize();
    devStatus1 = mpu2.dmpInitialize();


    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(220);
    mpu1.setYGyroOffset(76);
    mpu1.setZGyroOffset(-85);
    mpu1.setZAccelOffset(1788); // 1688 factory default for my test chip

    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788); // 1688 factory default for my test chip


    // make sure it worked (returns 0 if so)
    if (devStatus1 == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);
        mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus1 = mpu1.getIntStatus();
        mpuIntStatus2 = mpu2.getIntStatus();


        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize1 = mpu1.dmpGetFIFOPacketSize();
        packetSize2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus1);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus1 = mpu1.getIntStatus();
    mpuIntStatus2 = mpu2.getIntStatus();

    // get current FIFO count
    fifoCount1 = mpu1.getFIFOCount();
    fifoCount2 = mpu2.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    
    
    if ((mpuIntStatus1 & 0x10) || fifoCount1 == 1024) {
        // reset so we can continue cleanly
        mpu1.resetFIFO();
        mpu2.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus1 & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount1 < packetSize1) fifoCount1 = mpu1.getFIFOCount();

        // read a packet from FIFO
        mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
        mpu2.getFIFOBytes(fifoBuffer2, packetSize2);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount1 -= packetSize1;
        fifoCount2 -= packetSize2;



        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
            mpu1.dmpGetGravity(&gravity1, &q1);
            mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
            
            long time=millis();       
            Serial.print(time); Serial.print("\t");

            
            Serial.print("ypr\t");
            Serial.print(ypr1[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr1[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr1[2] * 180/M_PI);
            
            mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
            mpu2.dmpGetGravity(&gravity2, &q2);
            mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
            Serial.print("\t");
            Serial.print(ypr2[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr2[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr2[2] * 180/M_PI);
            delay(500);
            
        #endif

 
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
