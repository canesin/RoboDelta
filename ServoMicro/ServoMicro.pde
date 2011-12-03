#include <Servo.h> 

// Create a Servo object for each servo
Servo servo1;
Servo servo2;
Servo servo3;

// Common servo setup values
int minPulse = 600;   // minimum servo position, us (microseconds)
int maxPulse = 2400;  // maximum servo position, us

// User input for servo and position
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos=0;             // servo angle 0-180
int i;               // iterator
int start=0;        // Start in upper position

// LED on Pin 13 for digital on/off demo
int ledPin = 13;
int pinState = LOW;

void setup() 
{ 
  // Attach each Servo object to a digital pin
  servo1.attach(5, minPulse, maxPulse);
  servo2.attach(6, minPulse, maxPulse);
  servo3.attach(7, minPulse, maxPulse);

  // LED on Pin 13 for digital on/off demo
  pinMode(ledPin, OUTPUT);

  // Open the serial connection, 9600 baud
  Serial.begin(9600);
} 

void loop() 
{ 
  
  if (start == 0) {
    servo1.write(0);
    servo2.write(0);
    servo3.write(0);
    delay(1500);    
  }
  // Wait for serial input (min 3 bytes in buffer)
  if (Serial.available() > 3) {
    start=1;
    // Read the first byte
    startbyte = Serial.read();
    // If it's really the startbyte (255) ...
    if (startbyte == 255) {
      // ... then get the next two bytes
      for (i=0;i<2;i++) {
        userInput[i] = Serial.read();
      }
      // First byte = servo to move?
      servo = userInput[0];
      // Second byte = which position?
      pos = userInput[1];
      // Packet error checking and recovery
      if (pos == 255) { servo = 255; }

      // Assign new position to appropriate servo
      switch (servo) {
        case 1:
          servo1.write(pos);    // move servo1 to 'pos'
          break;
        case 2:
          servo2.write(pos);
          break;
        case 3:
          servo3.write(pos);
          break;

        // LED on Pin 13 for digital on/off demo
        case 99:
          if (pos == 180) {
            if (pinState == LOW) { pinState = HIGH; }
            else { pinState = LOW; }
          }
          if (pos == 0) {
            pinState = LOW;
          }
          digitalWrite(ledPin, pinState);
          break;
      }
    }
  }
}

