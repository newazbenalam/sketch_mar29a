/*Obstacle avoidance robot with three ultrasonic sensors
 * https://srituhobby.com
 */
//Include the motor driver library
// #include <AFMotor.h>
//Define the sensor pins
#define S1Trig A0
#define S2Trig A1
#define S3Trig A2
#define S1Echo A3
#define S2Echo A4
#define S3Echo A5

#define IR 12
//Set the speed of the motors
#define Speed 160

//motor pinout
#define Red 7
#define Orange 6
#define Yellow 5
#define Green 4

//Create objects for the motors
// AF_DCMotor motor1(1);
// AF_DCMotor motor2(2);
// AF_DCMotor motor3(3);
// AF_DCMotor motor4(4);

void setup() {
  Serial.begin(9600);
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);
  //Set the speed of the motors
  // motor1.setSpeed(Speed);
  // motor2.setSpeed(Speed);
  // motor3.setSpeed(Speed);
  // motor4.setSpeed(Speed);
  pinMode(Red, OUTPUT);
  pinMode(Orange, OUTPUT);
  pinMode(Yellow, OUTPUT);
  pinMode(Green, OUTPUT);  
  pinMode(IR, INPUT);
}

void loop() {
  int centerSensor = sensorOne();
  int leftSensor = sensorThree();
  int rightSensor = sensorTwo();
  int SensorValue = digitalRead(IR);
// Check the distance using the IF condition
  // Serial.println(leftSensor);
  if ( centerSensor < 5) {
    Reverse();
    Serial.println("Reverse");
    delay(10); // 15
    int centerSensor = sensorOne();
    // Stop();
    return;
  }

  if (20 >= centerSensor ) { // 8 // 10 >= centerSensor || || SensorValue == LOW
    Stop();
    Serial.println("Stop");
    delay(15);

    if (leftSensor > rightSensor) {
      left();
      Serial.println("Left");
      delay(150); // 300
      return;
    } else {
      right();
      Serial.println("Right");
      delay(150);
      return;
    }
  }

  if (leftSensor < 6){
    right();
    Serial.println("Right");
    delay(10);
    // Serial.println(leftSensor);
    if (leftSensor < 3) {
      ReverseLeft();
      delay(15);
      Reverse();
      delay(15);
      return;
    }
  }
  // else {
  //   Serial.println("Forward");
  //   forward();
  // }
    
  if (rightSensor < 6){
    left();
    Serial.println("Left");
    delay(10); //40
    if (rightSensor < 3) {
      ReverseRight();
      delay(15); //15
      Reverse();
      delay(15); //15
      return;
    }
  }

  // Serial.println("Forward");
  forward();
}

//Get the sensor values
int sensorOne() {
  //pulse output
  digitalWrite(S1Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S1Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S1Trig, LOW);

  long t = pulseIn(S1Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

//Get the sensor values
int sensorTwo() {
  //pulse output
  digitalWrite(S2Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S2Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S2Trig, LOW);

  long t = pulseIn(S2Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

//Get the sensor values
int sensorThree() {
  //pulse output
  digitalWrite(S3Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S3Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S3Trig, LOW);

  long t = pulseIn(S3Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}

/*******************Motor functions**********************/
void forward() {
  // motor1.run(FORWARD);
  // motor2.run(FORWARD);
  // motor3.run(FORWARD);
  // motor4.run(FORWARD);
  digitalWrite(Red, HIGH);
  digitalWrite(Orange, HIGH);
  digitalWrite(Yellow, LOW);
  digitalWrite(Green, LOW);

}
void left() {
  // motor1.run(BACKWARD);
  // motor2.run(BACKWARD);
  // motor3.run(FORWARD);
  // motor4.run(FORWARD);
  digitalWrite(Red, LOW);
  digitalWrite(Orange, HIGH);
}
void right() {
  // motor1.run(FORWARD);
  // motor2.run(FORWARD);
  // motor3.run(BACKWARD);
  // motor4.run(BACKWARD);
  digitalWrite(Red, HIGH);
  digitalWrite(Orange, LOW);
}
void Stop() {
  // motor1.run(RELEASE);
  // motor2.run(RELEASE);
  // motor3.run(RELEASE);
  // motor4.run(RELEASE);
  digitalWrite(Red, LOW);
  digitalWrite(Orange, LOW);
  digitalWrite(Yellow, LOW);
  digitalWrite(Green, LOW);
}
void Reverse() {
  // motor1.run(RELEASE);
  // motor2.run(RELEASE);
  // motor3.run(RELEASE);
  // motor4.run(RELEASE);
  digitalWrite(Red, LOW);
  digitalWrite(Orange, LOW);
  digitalWrite(Yellow, HIGH);
  digitalWrite(Green, HIGH);
}

void ReverseRight() {
  digitalWrite(Red, LOW);
  digitalWrite(Orange, LOW);
  digitalWrite(Yellow, LOW);
  digitalWrite(Green, HIGH);
}

void ReverseLeft() {
  digitalWrite(Red, LOW);
  digitalWrite(Orange, LOW);
  digitalWrite(Yellow, HIGH);
  digitalWrite(Green, LOW);
}
