/*
  Software serial multiple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 10); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(4800);
  mySerial.println("Hello, world?");
}

void loop() { // run over and over
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}

/*  BASIC SERVO CONTROL

    SoftwareSerial mySerial(9,10);  // Rx, Tx
Servo tilt, pan; // create servo object to control a servo 

int joyX = A0; // analog pin used to connect the X - axis of Joystick
int joyY = A1; // analog pin used to connect the Y - axis of Joystick
int x, y; // variables to read the values from the analog pins 

void setup()
{ 
  tilt.attach(9); // attaches the tilt servo on pin 9 to the servo object 
  pan.attach(10); // attaches the pan servo on pin 10 to the servo object 
} 

void loop()
{ 
  x = joyX;    // reads the value of the Joystick's X - axis (value between 0 and 1023) 
  y = joyY;    // reads the value of the Joystick's Y - axis (value between 0 and 1023) 
  x = map(analogRead(joyX), 0, 1023, 900, 2100); // scale it to use with the servo b/w 900 usec to 2100 usec
  y = map(analogRead(joyY), 0, 1023, 900, 2100);
  tilt.write(x); // sets the servo position according to the scaled value 
  pan.write(y);
  delay(15); // waits for the servos to get there 
}


/* ANOTHER BASIC SERVO CONTROL

const int servo1 = 3;       // first servo
const int servo2 = 10;       // second servo
const int joyH = 3;        // L/R Parallax Thumbstick
const int joyV = 4;        // U/D Parallax Thumbstick

int servoVal;           // variable to read the value from the analog pin

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo



void setup() {

  // Servo  
  myservo1.attach(servo1);  // attaches the servo
  myservo2.attach(servo2);  // attaches the servo

  // Inizialize Serial
  Serial.begin(9600);
}


void loop(){

    // Display Joystick values using the serial monitor
    outputJoystick();

    // Read the horizontal joystick value  (value between 0 and 1023)
    servoVal = analogRead(joyH);          
    servoVal = map(servoVal, 0, 1023, 0, 180);     // scale it to use it with the servo (result  between 0 and 180)

    myservo2.write(servoVal);                         // sets the servo position according to the scaled value    

    // Read the horizontal joystick value  (value between 0 and 1023)
    servoVal = analogRead(joyV);           
    servoVal = map(servoVal, 0, 1023, 70, 180);     // scale it to use it with the servo (result between 70 and 180)

    myservo1.write(servoVal);                           // sets the servo position according to the scaled value

    delay(15);                                       // waits for the servo to get there

}


/**
* Display joystick values

void outputJoystick(){

    Serial.print(analogRead(joyH));
    Serial.print ("---"); 
    Serial.print(analogRead(joyV));
    Serial.println ("----------------");
}

*/
