
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

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

#include <SPP.h>  // (for android controller)

#include <Encoder.h>
#include <Wii.h>
#include <usbhub.h>
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif





USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD btBtd(&Usb);
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
WII Wii(&Btd, PAIR); // This will start an inquiry and then pair with your Wiimote - you only have to do this once
//WII Wii(&Btd); // After that you can simply create the instance like so and then press any button on the Wiimote

SPP SerialBT(&btBtd); // This will set the name to the defaults: "Arduino" and the pin to "0000"
//SPP SerialBT(&Btd, "Lauszus's Arduino", "1234"); // You can also set the name and pin like so
//SPP SerialBT; 

unsigned long btstarttime;
unsigned long btendtime;
unsigned long btperiod =0;
int btignore=0;



boolean firstMessage = true; //for android bluetooth

bool printAngle;

char ReceiveByte; //for the android bluetooth spp


#define trigPin 47
#define echoPin 49


#include <PID_v1.h>
#include "DualVNH5019MotorShield.h" //motor library jc

DualVNH5019MotorShield md; //motor include jc
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=75, aggKi=280, aggKd=1.9;
double consKp=80, consKi=1500 , consKd=3.9;

//double aggKp=70, aggKi=240, aggKd=1.9;
//double consKp=80, consKi=250 , consKd=2.1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);



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
MPU6050 mpu;


Encoder myEnc(18, 17);//set pins for encoder (interrupt pins
Encoder myEnc2(16,19);
signed long currenttime, starttime, period, currentclick, startclick, distance;




int turnsignal; //for turning

//MPU6050 mpu(0x69); // <-- use for AD0 high

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
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

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
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

float mycurrentangle;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



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
  
  
  btstarttime=micros();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  starttime=0;
  startclick=0;
  
    //initialize the variables we're linked to
  Input = mycurrentangle;
  Setpoint =5.5;
  
   // myPID.SetOutputLimits(-10000,10000);
    myPID.SetOutputLimits(-400,400);
    
    myPID.SetSampleTime(1);


  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  
    
  
  
  
   md.init(); //motor thing initialize
   
   
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
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

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
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(1, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
   //  SerialBT = new SPP(&Btd);  
      while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nWiimote Bluetooth Library Started"));
    
    
}




long oldPosition  = -999;   //this is for the encoder





// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
 while (!mpuInterrupt && fifoCount < packetSize) {

   
   
  
 // turnsignal=0; //for reseting turn signal
       double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<1)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
    Input = mycurrentangle;
    if(btignore==0)
    {
      turnsignal=0;
      Setpoint=6;
    //  Serial.print("Set");
    }
    
  //  readsensor();
   readspeed();
    readremotewii(); 
    readrealremotewii(); 
   
          
   
  //   if(Setpoint==12)
  //  {
 //     Serial.println("ASDFG");
 //   }
    
 //   if(Setpoint !=12)
 //   {
      
  //  readandroid();
  //  Serial.print(Setpoint);
 //   if (mycurrentangle>45 || mycurrentangle<-45)
 // {
   
  //  md.setSpeeds(0,0);   
  //  delay(10000);
  //}
    myPID.Compute(); 
 //  if(Setpoint==12)
  //  {
 //     Serial.println("bnm");
 //   }
    if(turnsignal==1)
    {
      md.setSpeeds(Output+150,Output-150);
     // Serial.println("motor turning");
    }
    if(turnsignal==2)
    {
      md.setSpeeds(Output-150,Output+150);
    }
    else if(turnsignal==0);
    {
     md.setSpeeds(Output,Output);   
    }
 //   if(Setpoint==12)
  //  {
  //    Serial.println("cvx");
  //  }
     //  Serial.println(Output);
     //  Serial.println("mycurrentvalue");


    } 

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         /*   Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
          */
            mycurrentangle=(ypr[1] * 180/M_PI);
         //   Serial.print(mycurrentangle);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
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

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/*
void readremotewii ()
{
  int BTinput;
      Usb.Task(); 
      
       if (SerialBT->connected) 
       {
    //  Setpoint=12;
        if (SerialBT->available())
             {
               //   Setpoint=12;
             BTinput = SerialBT->read();
                    Serial.write(BTinput);
                    if( BTinput == 'F' )
                    {
                        Serial.println("fwd");
                        Setpoint=12;
                      //  delay(50000);
                    }
                    if( BTinput == 'B' )
                        Serial.println("backward");
                    if( BTinput == 'L' )
                        Serial.println("left");
                    if( BTinput == 'R' )
                        Serial.println("right");
             }
      }
  
  */
  
  
  void readremotewii ()
{
  int changesetpoint=0;
  int BTinput;
      Usb.Task(); 
      
     if (SerialBT.connected) 
       {
   // Serial.println("v");
        if (SerialBT.available())
             {
             //  Serial.println("w");
             BTinput = SerialBT.read();
                  //  Serial.write(BTinput);
                    if( BTinput == 'F' )
                    {
                        btstarttime=micros();
                        btignore=1;
                       // Serial.println("xxfwd");
                        Setpoint=8;
                     //   changesetpoint=1;
                      //  Serial.println(Setpoint);
                      //  delay(50000);
                    }
                    else if(BTinput=='B')
                    {
                      btstarttime=micros();
                      btignore=1;
                      Setpoint=5;
                    }
                    else if(BTinput=='R')
                    {
                      btstarttime=micros();
                      btignore=1;
                      turnsignal=1;
                    }
                    else if(BTinput=='L')
                    {
                      btstarttime=micros();
                      btignore=1;
                      turnsignal=2;
                    }
                   
                    else
                    {
                      btendtime=micros();
                      btperiod=btendtime-btstarttime;
                    //  Serial.println(btperiod);
                      if(btperiod<10000)
                      {
                        btignore=1;
                      }
                      else
                      {
                        btignore=0;
                      }
                    }
                
             }
      }
}

 // if(changesetpoint==1)
 // {
   // Serial.println(Setpoint);
 // }
  
 // Serial.println("y");
 // Serial.println(Setpoint);
  
  
  
/*

  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (SerialBT.connected) {
    if (firstMessage) {
      firstMessage = false;
      SerialBT.println(F("Hello from Arduino")); // Send welcome message
    }
  //  if (Serial.available())
  //    SerialBT.write(Serial.read());
    if (SerialBT.available()){
      ReceiveByte = SerialBT.read();
      Serial.write(ReceiveByte);
      Setpoint=8;
      if(ReceiveByte == 'F')
      {
        Serial.println("forward");
        Setpoint=8;
      }else if(ReceiveByte== 'B')
      {
        Setpoint= 4;
      }else if(ReceiveByte== 'L')
      {
        turnsignal=2;
      }else if(ReceiveByte== 'R')
      {
        turnsignal=1;
      }
    }
  }
  else
    firstMessage = true;
*/
  
  
  
  
void readrealremotewii()
{
 Usb.Task();
 
  if (Wii.wiimoteConnected) {
    if (Wii.getButtonClick(HOME)) { // You can use getButtonPress to see if the button is held down
      Serial.print(F("\r\nHOME"));
      Wii.disconnect();
    }
    else {
      if (Wii.getButtonPress(LEFT)) {
        turnsignal=2;
        Serial.print(F("\r\nLeft"));
        }
      
      if (Wii.getButtonPress(RIGHT)) {
       turnsignal=1;
      
        Serial.print(F("\r\nRight"));
      }
      if (Wii.getButtonPress(DOWN)) {
       Setpoint=4;
        Serial.print(F("\r\nDown"));
      }
      if (Wii.getButtonPress(UP)) {
       
        Serial.print(F("\r\nUp"));
      }

      if (Wii.getButtonClick(PLUS))
        Serial.print(F("\r\nPlus"));
      if (Wii.getButtonClick(MINUS))
        Serial.print(F("\r\nMinus"));

      if (Wii.getButtonClick(ONE))
        Serial.print(F("\r\nOne"));
      if (Wii.getButtonClick(TWO))
        Serial.print(F("\r\nTwo"));

      if (Wii.getButtonClick(A)) {
        printAngle = !printAngle;
        Serial.print(F("\r\nA"));
      }
      if (Wii.getButtonPress(B)) {
        Setpoint=8;
        
       // Wii.setRumbleToggle();
        Serial.print(F("\r\nB"));
      }
    }
    if (printAngle) {
      Serial.print(F("\r\nPitch: "));
      Serial.print(Wii.getPitch());
      Serial.print(F("\tRoll: "));
      Serial.print(Wii.getRoll());
      if (Wii.motionPlusConnected) {
        Serial.print(F("\tYaw: "));
        Serial.print(Wii.getYaw());
      }
      if (Wii.nunchuckConnected) {
        Serial.print(F("\tNunchuck Pitch: "));
        Serial.print(Wii.getNunchuckPitch());
        Serial.print(F("\tNunchuck Roll: "));
        Serial.print(Wii.getNunchuckRoll());
      }
    }
  }
  if (Wii.nunchuckConnected) {
    if (Wii.getButtonClick(Z))
      Serial.print(F("\r\nZ"));
    if (Wii.getButtonClick(C))
      Serial.print(F("\r\nC"));
    if (Wii.getAnalogHat(HatX) > 137 ||  Wii.getAnalogHat(HatX) < 117 || Wii.getAnalogHat(HatY) > 137 || Wii.getAnalogHat(HatY) < 117) {
      Serial.print(F("\r\nHatX: "));
      Serial.print(Wii.getAnalogHat(HatX));
      Serial.print(F("\tHatY: "));
      Serial.print(Wii.getAnalogHat(HatY));
    }
  }  
} 
  


void readspeed()
{
 
  currenttime=micros();
 //  Serial.println(currenttime);
 period= currenttime-starttime;
 if(period>=10000)
 {
 // currentclick=myEnc.read();
   currentclick=myEnc.read(); //+myEnc2.read();
 
 
 
   //Serial.println(currentclick);
  distance=currentclick-startclick;
  
  
         


 // if(!Wii.getButtonPress(DOWN) || !Wii.getButtonPress(B))
 // {
  // long newPosition = myEnc.read();
 //  if (newPosition != oldPosition) 
 //  {
  //   oldPosition = newPosition;
  //   Serial.println(newPosition);
   
 //    if(newPosition>300)
 //    {
  //    Setpoint=7.5;
 //     Serial.println("setpoint 7");
 //    }
 //    if(newPosition<-300)
 //    {
  //    Setpoint=4.5;
  //    Serial.println("setpoint 4.5");
  //   }
  // }  
            Serial.println(distance);
            if(distance >60)
            {
              Setpoint=11.5;
              Serial.println("setpoint 7");
            }
            if(distance<-60)
            {
              Setpoint=.5;
              Serial.println("setpoint 5");
            }  
                 startclick=currentclick;
              starttime=currenttime;
                   
   }       
         
}


void readsensor() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(5); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

 
  if (distance >= 200 || distance <= 0){
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
 // delay(5);
}


/*
void readandroid() {
  Usb.Task(); // The SPP data is actually not send until this is called, one could call SerialBT.send() directly as well

  if (SerialBT.connected) {
    if (firstMessage) {
      firstMessage = false;
      SerialBT.println(F("Hello from Arduino")); // Send welcome message
    }
    if (Serial.available())
      SerialBT.write(Serial.read());
    if (SerialBT.available())
      Serial.write(SerialBT.read());
  }
  else
    firstMessage = true;
}
*/
