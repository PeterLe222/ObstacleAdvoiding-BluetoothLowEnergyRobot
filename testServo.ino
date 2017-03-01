#include <Servo.h>
#include "AFMotor.h"

Servo servo;
AF_DCMotor MOTOR_1 (3, MOTOR12_64KHZ); // define motor on channel 3 with 64KHz PWM
AF_DCMotor MOTOR_2 (4, MOTOR12_64KHZ); // define motor on channel 4 with 64KHz PWM


boolean buttonState = LOW; 

volatile int pressed = 0;
const int BUTTONPIN = 2;
const int TRIG = A0;  // trig pin of Ultrasonic Sensor
const int ECHO = A1;  // echo pin of Ultrasonic Sensor
char state;

boolean debounceButton(boolean state);
void buttonCount();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(10);
  pinMode(BUTTONPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonCount, RISING);
  MOTOR_1.setSpeed(255);  // set the max speed for motor 1
  MOTOR_2.setSpeed(255);  // set the max speed for motor 2
  pinMode(TRIG, OUTPUT);  // trig sends the signal
  pinMode(ECHO, INPUT);   // echo receives the signal  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(debounceButton(buttonState) == LOW && buttonState == HIGH)
    {
      buttonState = LOW;
    } 
    
  Serial.println(pressed);
  if (pressed == 1)
  {
    MOTOR_1.run(RELEASE);
    MOTOR_2.run(RELEASE);
  }
  if (pressed == 2)
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
  if (pressed == 0)
  {
    unsigned long duration; // store variable for the calculation of time
    int distance;           // store variable for the distance
    for (int i = 45; i <= 135; i++)
    {
      Serial.println(i);
      servo.write(i);
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
    
      /* robot runs backward and turn right
         if the robot is near object */
      if (distance < 15)
      {
        MOTOR_1.run(BACKWARD);  // motor 1 run backward
        MOTOR_2.run(BACKWARD);  // motor 2 run backward
        delay(1000);            // run backward in 1 second
        MOTOR_1.run(BACKWARD);  
        MOTOR_2.run(FORWARD);   // motor 2 run forward
        delay(500); 
      }
      /* robot run forward or be stopped 
         until stuck reaches 800 */
      else
      {
        MOTOR_1.run(FORWARD);
        MOTOR_2.run(FORWARD);
      }
    }
  
    for (int i = 135; i >= 45; i--)
    {
      servo.write(i);
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
    
      /* robot runs backward and turn right
         if the robot is near object */
      if (distance < 15)
      {
        MOTOR_1.run(BACKWARD);  // motor 1 run backward
        MOTOR_2.run(BACKWARD);  // motor 2 run backward
        delay(1000);            // run backward in 1 second
        MOTOR_1.run(BACKWARD);  
        MOTOR_2.run(FORWARD);   // motor 2 run forward
        delay(500); 
      }
      /* robot run forward or be stopped 
         until stuck reaches 800 */
      else
      {
        MOTOR_1.run(FORWARD);
        MOTOR_2.run(FORWARD);
      }
    }
  }
  if (pressed == 3)
  {
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



