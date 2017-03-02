/*  Robot has 2 modes: 
 *   - Controlling vie Bluetooth Low Energy
 *   - Obstacle Advoiding
 */
 
#include <Servo.h>
#include "AFMotor.h"

Servo servo;  // call Servo from Servo library

AF_DCMotor MOTOR_1 (3, MOTOR12_64KHZ); // define motor on channel 3 with 64KHz PWM
AF_DCMotor MOTOR_2 (4, MOTOR12_64KHZ); // define motor on channel 4 with 64KHz PWM


boolean buttonState = LOW; /* variable store status of 
                              the button is pressed or not */
volatile int pressed = 0; // variable store number of pressed button
const int BUTTONPIN = 2;  // pin 2 connects to button
const int TRIG = A0;  // trig pin of Ultrasonic Sensor
const int ECHO = A1;  // echo pin of Ultrasonic Sensor
char state; // variable store the BLE's message receive from sender

boolean debounceButton(boolean state);  // function debounce the button
void buttonCount(); // function increase the number when the button is pressed
void bluetoothLowEnergy();  // function control the robot via BLE
void autoRun(); // function avoid objects 

void setup() {
  Serial.begin(9600);
  servo.attach(10); // attach servo to pin 10
  pinMode(BUTTONPIN, INPUT);  // set pin 2 as INPUT 
  /* Interrupt the button when the button is pressed */
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonCount, RISING);
  MOTOR_1.setSpeed(255);  // set the max speed for motor 1
  MOTOR_2.setSpeed(255);  // set the max speed for motor 2
  pinMode(TRIG, OUTPUT);  // trig sends the signal
  pinMode(ECHO, INPUT);   // echo receives the signal  
}

void loop() {

  /* change the buttonState to LOW when the button is pressed */
  if(debounceButton(buttonState) == LOW && buttonState == HIGH)
  {
    buttonState = LOW;
  } 
  Serial.println(pressed);  // print the number of pressed

  /* BLE control mode */
  if (pressed == 1)
  {
    bluetoothLowEnergy();
  }
  
  /* obstacle avoiding mode */
  if (pressed == 2)
  {
    // rotate the servo from 45 to 135 degree
    for (int i = 45; i <= 135; i++)  
    {
      servo.write(i); // change the servo degree
      autoRun();  // robot run automatically to avoid object
    }
    // rotate the servo from 135 to 45 degree
    for (int i = 135; i >= 45; i--)
    {
      servo.write(i);
      autoRun();
    }
  }

  /* turn off the Robot */
  if (pressed == 3)
  {
    MOTOR_1.run(RELEASE);
    MOTOR_2.run(RELEASE);
    pressed = 0;
  }
}

void buttonCount()
{
  if(debounceButton(buttonState) == HIGH && buttonState == LOW)
    {
      pressed++;
      Serial.println(pressed);
      buttonState = HIGH;
    }  
}

boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(BUTTONPIN);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(BUTTONPIN);
  }
  return stateNow;
}

void bluetoothLowEnergy() 
{
  state = Serial.read(); // recieve command from bluetooth
  Serial.println(state);
  // go forward
  if (state == 'a' || state == 'A'|| state == 'e' || state == 'E')
  {
    MOTOR_1.run(FORWARD);
    MOTOR_2.run(FORWARD);
  }
  // go backward
  if (state == 'c' || state == 'C'|| state == 'g' || state == 'G')
  {
    MOTOR_1.run(BACKWARD);
    MOTOR_2.run(BACKWARD);
  }
  // turn left
  if (state == 'd' || state == 'D'|| state == 'h' || state == 'H')
  {
    MOTOR_1.run(FORWARD);
    MOTOR_2.run(BACKWARD);
  }
  // turn right
  if (state == 'b' || state == 'B'|| state == 'f' || state == 'F')
  {
    MOTOR_2.run(FORWARD);
    MOTOR_1.run(BACKWARD);
  }
  // stop the robot
  if (state == 0)
  {
    MOTOR_1.run(RELEASE);
    MOTOR_2.run(RELEASE);
  }
}

void autoRun()
{
  unsigned long duration; // store variable for the calculation of time
  int distance;           // store variable for the distance  
  
  /* send signal from trig */
  digitalWrite(TRIG,0); // turn off pin trig
  delay(2);
  digitalWrite(TRIG,1); // send signal
  delay(5);
  digitalWrite(TRIG,0); // turn off pin trig

  /* calculate the time and distance */
  duration = pulseIn(ECHO,HIGH);      // calculate the length of the pulse
  distance = int(duration/2/29.412);  // calculate the distance
  Serial.print(distance);Serial.println("cm");

  /* robot runs backward in 1 second and turn right
     if the distance from robot to object smaller than 15 cm */
  if (distance < 15)
  {
    MOTOR_1.run(BACKWARD);  // motor 1 run backward
    MOTOR_2.run(BACKWARD);  // motor 2 run backward
    delay(1000);            // run backward in 1 second
    MOTOR_1.run(BACKWARD);  
    MOTOR_2.run(FORWARD);   // motor 2 run forward
    delay(500); 
  }
  /* robot run forward */
  else
  {
    MOTOR_1.run(FORWARD);
    MOTOR_2.run(FORWARD);
  }
}


