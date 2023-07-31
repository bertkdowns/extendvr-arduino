#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if IDCDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
// This is my old script from my 3d printed finger tracked controller!
// these store the valuse for finger tracking
byte pinkie = 0;
byte ring = 0;
byte middle = 0;
byte indexf = 0;
// the thumb byte works differently, storing individual values for what pins are
// on and what are off
byte thumb = 0;
unsigned long time;

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
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
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-28);
    mpu.setYGyroOffset(-46);
    mpu.setZGyroOffset(-18);
    mpu.setXAccelOffset(6468);
    mpu.setYAccelOffset(6634);
    mpu.setZAccelOffset(9229);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
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
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    
    time = millis();
}

// main program loop
void loop(){
  // get version number
  if(Serial.read() == 'v'){
    Serial.write((byte) 1);  
  }
  // if programming failed, dont try to do anything
  if(!dmpReady) return;
  // check if more mpu data to process
  if(!mpuInterrupt && fifoCount < packetSize) {
    // no more data to process, get the latest hand tracking data
    getHandTrackingData();
  } else {
    // new IMU data present, so get that and then send all the data to the phone/computer
    getMPUData();
    sendData();
  }
    
}
void getHandTrackingData(){
  pinkie = 0;
  ring = 0;
  middle = 0;
  indexf = 0;
  thumb = 0;
  // iterate through the matrix, getting moisture sensor values for each finger
  for(byte i = 3; i < 9; i++){
    // low the previous pin
    if(i == 3) digitalWrite(8,HIGH);
    else digitalWrite(i-1,HIGH);
    // high new pin we are going to read from
    digitalWrite(i,LOW);
    delay(1);
    // read the values, and if they are greater than 4 then
    // set the appropriate variable to whatever number across we are reading
    if(analogRead(0) < 1015) pinkie = (i-2);
    if(analogRead(1) < 1015) ring   = (i-2);
    if(analogRead(2) < 1015) middle = (i-2);
    if(analogRead(3) < 1015) indexf  = (i-2);
    // for the thumb, we set different bits in the byte to say what 
    // one we are changing
    if(analogRead(6) < 1015) thumb = thumb + getPower2(i-3);
  }
}

byte getPower2(byte power){
  byte result = 1;
  for(byte i = 0; i < power; i++){
    result = result * 2;
  }
  return result;
}



//////////////////////////////////////////////////////////////////
// this function gets any new rotation data from the mpu6050
//////////////////////////////////////////////////////////////////
void getMPUData(){
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
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetGravity(&gravity,&q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // ypr variable stores the yaw pitch roll data.
  }
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
  // write the integers to the serial
  Serial.write(highByte(w));Serial.write(lowByte(w));
  Serial.write(highByte(x));Serial.write(lowByte(x));
  Serial.write(highByte(y));Serial.write(lowByte(y));
  Serial.write(highByte(z));Serial.write(lowByte(z));
  Serial.write(thumb);
  Serial.write(indexf);
  Serial.write(middle);
  Serial.write(ring);
  Serial.write(pinkie);
  Serial.println();
  // reset time
  time = millis();
}
