#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if IDCDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// Script for the controller
// basically all you need to worry about is the getHandTrackingData function, which
// reads inputs/buttons/sensors and stores them in variables, and the 
// sendData function, which sends the data over bluetooth.
// also note the accellerometer/gyro offsets in the setup function: these have to be calibrated
// with a calibration script

// ints storing the analogRead values of the pins
int pinkie = 0;
int ring = 0;
int middle = 0;
int index = 0;
int thumb = 0;

// booleans storing if the fingers are 'up' or 'down'
bool pinkieDown = false;
bool ringDown = false;
bool middleDown = false;
bool indexDown = false;
bool thumbDown = false;


// up and down threshold: the up threshold is the minimum analog reading required to be certain the finger is up;
// the down threshold is the maximum analog reading required to be certain the finger is down.
// any values between the two we assume to be 'static' i.e random variations, and ignore (leave the state unchanged)
#define UP_THRESHOLD 1022
#define DOWN_THRESHOLD 1018





// Time since last sent readings to computer
unsigned long time;



// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container



////////////////////////////////////////////////////////////////////
// MPU INITALISATION AND READING STUFF
// you shouldn't really have to worry about any of this
////////////////////////////////////////////////////////////////////

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { // interrupt detection routine
    mpuInterrupt = true;
}
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize Serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(57600);
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


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // GYRO OFFSETS
    // to calibrate the MPU-6050
    // Use a MPU-6050 calibration script to get the offsets
    mpu.setXGyroOffset(-28);
    mpu.setYGyroOffset(-46);
    mpu.setZGyroOffset(-18);
    mpu.setXAccelOffset(6468);
    mpu.setYAccelOffset(6634);
    mpu.setZAccelOffset(9229);
    
    
    if (devStatus == 0) {
        // successful initiation
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    time = millis();
}


void getMPUData(){

  // this function gets any new rotation data from the mpu6050

  
  //reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow ( this should never happen unless our code is too inefficeint)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
    // reset co we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  // otherwise, check forDMP data ready interrupt (this should happen frequently) 
  }else if(mpuIntStatus & 0x02){
    // wait for corrent available data length, should be a VERY short wait
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track fifo count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // write quaternion to buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
  }
}

/////////////////////////////////////////////////////
// Main Program loop!
// This bit and after is more important
/////////////////////////////////////////////////////
void loop(){
  // Allow getting version number - for future use if we have different versions of controllers running at the same time
  if(Serial.read() == 'v') Serial.write((byte) 1);  
  
  
  // if programming failed, dont try to do anything
  if(!dmpReady) return;
  
  // CHECK IF NEW IMU DATA TO PROCESS
  if(mpuInterrupt || fifoCount >= packetSize){
    // new IMU data present!
    // get IMU and hand tracking data, send it to phone/computer
    getMPUData();
    getHandTrackingData();
    sendData();
  } 
}


void getHandTrackingData(){
  // store the positions of each finger
  pinkie = analogRead(0)
  ring = analogRead(1);
  middle = analogRead(2);
  index = analogRead(3);
  thumb = analogRead(4);
  // Figure out if the positions of the fingers have changed: if they have update the state
  pinkieDown = pinkie < DOWN_THRESHOLD ? false : pinkie > UP_THRESHOLD ? true : pinkie;
  ringDown   = ring   < DOWN_THRESHOLD ? false : ring   > UP_THRESHOLD ? true : ring  ;
  middleDown = middle < DOWN_THRESHOLD ? false : middle > UP_THRESHOLD ? true : middle;
  indexDown  = index  < DOWN_THRESHOLD ? false : index  > UP_THRESHOLD ? true : index ;
  thumbDown  = thumb  < DOWN_THRESHOLD ? false : thumb  > UP_THRESHOLD ? true : thumb ;
  
}




//////////////////////////////////////////////////
// send data via serial
//////////////////////////////////////////////////
void sendData(){
  
  // send data a maximum of 60fps (1000/60 = about 16 milliseconds)
  if(time + 16 > millis()){
    // we haven't waited 16 milliseconds yet, don't bother sending new readings
    return;
  }
  
  // display yaw pitch roll values, and finger values
  int w = 1000 * q.w;
  int x = 1000 * q.x;
  int y = 1000 * q.y;
  int z = 1000 * q.z;
  // write the quaternion as integers to serial
  Serial.write(highByte(w));Serial.write(lowByte(w));
  Serial.write(highByte(x));Serial.write(lowByte(x));
  Serial.write(highByte(y));Serial.write(lowByte(y));
  Serial.write(highByte(z));Serial.write(lowByte(z));
  // finger data is designed to be a number be a number between 0 and 255 corresponding to how much the finger is bent
  // all 255 = closed fist, all 0 = open fist
  // this is for if future versions have more accurate tracking than this version
  Serial.write(thumbDown ?255b:0b);
  Serial.write(indexDown?255b:0b);
  Serial.write(middleDown?255b:0b);
  Serial.write(ringDown  ?255b:0b);
  Serial.write(pinkieDown?255b:0b);
  Serial.println();
  // reset time
  time = millis();
}
