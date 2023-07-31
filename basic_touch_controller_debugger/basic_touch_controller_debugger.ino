

// Script for the controller
// basically all you need to worry about is the getHandTrackingData function, which
// reads inputs/buttons/sensors and stores them in variables, and the 
// sendData function, which sends the data over bluetooth.
// also note the accellerometer/gyro offsets in the setup function: these have to be calibrated
// with a calibration script


int pinkie = false;
int ring = false;
int middle = false;
int indexf = false;
int thumb = false;


void setup() {
    Serial.begin(9600);
}

// main program loop
void loop(){
  pinkie = analogRead(0);
  ring = analogRead(1);
  middle = analogRead(2);
  indexf = analogRead(3);
  thumb = analogRead(6);
  Serial.print(thumb);Serial.print("\t");
  Serial.print(indexf);Serial.print("\t");
  Serial.print(middle);Serial.print("\t");
  Serial.print(ring);Serial.print("\t");
  Serial.print(pinkie);Serial.print("\n");
  delay(100);

}
