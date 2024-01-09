#include <Servo.h>

Servo stroke;  // create servo object to control a servo
Servo fin1;
Servo fin2;
// twelve servo objects can be created on most boards

int stroke_pos = 0;    // variable to store the servo position
int fin1_pos = 0;
int fin2_pos = 0;

void setup() {
  // put your setup code here, to run once:
  stroke.attach(9);  // attaches the servo on pin 9 to the servo object
  fin1.attach(10);
  fin2.attach(11);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  // TODO: add arrow controls of stroke & fins
  for (stroke_pos = 0; stroke_pos <= 180; stroke_pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    stroke.write(pos);              // tell servo to go to position in variable 'pos'
    //fin1.write(180 - pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (stroke_pos = 180; stroke_pos >= 0; stroke_pos -= 1) { // goes from 180 degrees to 0 degrees
    stroke.write(pos);              // tell servo to go to position in variable 'pos'
    //fin1.write(180 - pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
