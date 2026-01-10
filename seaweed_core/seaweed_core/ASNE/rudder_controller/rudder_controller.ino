// This code is for controlling a servo motor
// controller using an arduino nano every. The
// Jetson Nano will be sending angles to the arduino
// over serial USB.

// When receiveing an angle from the Jetson,
// these are relative values from the home
// position. Meaning '0' is considered at home, '1'
// is one degree counterclockwise, '-1' is
// one degreee clockwise, etc.)

#include <Servo.h>
Servo myServo;
int raw_angle;
int new_angle;

void setup() {
  // use arduino pin 9 for PWM output with a 
  // minimum pulse width value of 500us and 
  // a maximum pulse width value of 2500us 
  myServo.attach(14, 500, 2500);

  // send the rudder to home before receiving
  // inputs from jetson
  myServo.write(90);

  // open serial communication channel for arduino
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void  loop() {
  while (Serial.available()){
   // receive angle from jetson
    // raw_angle = Serial.readString().toInt();
    raw_angle = Serial.parseInt();

    // process received angle for arduino
    new_angle = convertToRange(raw_angle);

    // generate PWM output for motor controller
    myServo.write(new_angle);
    delay(100);
  }
}

//*********************************************************
// convertToRange processes the angle received from Jetson
// and converts it to the correct angle for the servo motor 
//*********************************************************
int convertToRange(int raw_angle) {
    return raw_angle + 90;
}