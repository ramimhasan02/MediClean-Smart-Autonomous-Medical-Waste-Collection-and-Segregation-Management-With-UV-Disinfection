// Libraries
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// sonar
#define trigPinf 3
#define echoPinf 2
#define trigPinl 22
#define echoPinl 23
#define trigPinr 24
#define echoPinr 25

#define trigPinb1 26
#define echoPinb1 27
#define trigPinb2 28
#define echoPinb2 29
#define trigPinb3 30
#define echoPinb3 31
#define trigPinb4 32
#define echoPinb4 33
#define trigPinb5 34
#define echoPinb5 35
#define trigPinb6 36
#define echoPinb6 37

// servo
Servo myservo;
Adafruit_MPU6050 mpu;
float distancef;
float distancel;
float distancer;
float z_angle = 0;
int count = 0;
int corner = 0;

// motors
int ENA = 5;
int ENB = 6;
int spd = 100; // range: 0 to 255
float rot = 80;
//int mop_cw = 28;
//int mop_ccw = 29;
//int mop_speed = 11;

#define Right_R_PWM 7 // Right motor forward
#define Right_L_PWM 8 // Right motor backward

// Left Motor Control Connections
#define Left_R_PWM 9 // Left motor forward
#define Left_L_PWM 10 // Left motor backward

void forward()
{
    // Motor forward
    analogWrite(Right_R_PWM,120);
    analogWrite(Right_L_PWM, 0);
    analogWrite(Left_R_PWM, 120);
    analogWrite(Left_L_PWM, 0);

  
}
void backward()
{
    // Motor forward
    analogWrite(Right_R_PWM,0);
    analogWrite(Right_L_PWM, 120);
    analogWrite(Left_R_PWM, 0);
    analogWrite(Left_L_PWM, 120);

  
}
void left()
{
    // Motor forward
    analogWrite(Right_R_PWM,120);
    analogWrite(Right_L_PWM, 0);
    analogWrite(Left_R_PWM, 0);
    analogWrite(Left_L_PWM, 120);

  
}
void right()
{
    // Motor forward
    analogWrite(Right_R_PWM,0);
    analogWrite(Right_L_PWM, 120);
    analogWrite(Left_R_PWM, 120);
    analogWrite(Left_L_PWM, 0);

  
}

void stp()   //motor stop
{  
  
    analogWrite(Right_R_PWM, 0);
    analogWrite(Right_L_PWM, 0);
    analogWrite(Left_R_PWM, 0);
    analogWrite(Left_L_PWM, 0); 
   
  }


int calcdisf() // measures the distance ahead
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinf, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinf, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinf, LOW);

  duration = pulseIn(echoPinf, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisl() // measures the distance in left
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinl, LOW);

  duration = pulseIn(echoPinl, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisr() // measures the distance in right
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinr, LOW);

  duration = pulseIn(echoPinr, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

// Function to measure distance from a sonar
float calcdisb(int trigPin, int echoPin) {
  float duration, cm;
  delay(70);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  cm = (duration / 2) * 0.0343;
  return cm;
}

// Function to check if any sonar has distance less than 30 cm
bool checkSonarArray30(float distances[]) {
  for (int i = 0; i < 6; i++) {
    if (distances[i] < 30.0) {
      return true;
    }
  }
  return false;
}

// Function to check if any sonar has distance less than 10 cm
bool checkSonarArray10(float distances[]) {
  for (int i = 0; i < 6; i++) {
    if (distances[i] < 10.0) {
      return true;
    }
  }
  return false;
}

// Function to get the indices of sonars that have distance < 10 cm
void getSonarsBelowThreshold(float distances[], int activeSonars[]) {
  int index = 0;
  for (int i = 0; i < 6; i++) {
    if (distances[i] < 30.0) {
      activeSonars[index++] = i + 1;  // Store the sonar index (1-based)
    }
  }
  activeSonars[index] = -1;  // Mark the end of the list with -1
}


float angle() // measures the turning angle in degrees
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float Z_rotation;
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float dt = (float)(currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  /* Print out the values */
  Z_rotation = (g.gyro.z + 0.012) * 180 / 3.1416;

  z_angle = (z_angle + Z_rotation * dt);
  //Serial.print(z_angle);
  //Serial.println(" deg");
  return z_angle;
}

void zigzag() // function for navigating the room
{
  z_angle = 0;
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    forward();
    delay(350);
    stp();
    distancef = calcdisf();
    distancel = calcdisl();
    distancer = calcdisr();
    if((distancef<30)&&((distancel<30)||(distancer<30)))
    {
      corner=corner+1;
    }
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    forward();
    count = count + 1;
}

 int lookRight() // front sonar turns right and measures distance
{
    myservo.write(72); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122);
//    Serial.print(distance);
//    Serial.println(" cm"); 
    return distance;
}
int lookLeft()// front sonar turns left and measures distance
{
    myservo.write(172); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122); 
//    Serial.print(distance);
//    Serial.println(" cm");
    return distance;
    //delay(1000);
}
void mop()
{
  analogWrite(mop_speed, 50);
  digitalWrite(mop_cw, HIGH);
  digitalWrite(mop_ccw, LOW);
}

void stop_mop()
{
  digitalWrite(mop_cw, LOW);
  digitalWrite(mop_ccw, LOW);
}
void setup()
{
  //Serial.begin(9600);
  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(trigPinb1, OUTPUT); //sonar array begin
  pinMode(echoPinb1, INPUT);
  pinMode(trigPinb2, OUTPUT);
  pinMode(echoPinb2, INPUT);
  pinMode(trigPinb3, OUTPUT);
  pinMode(echoPinb3, INPUT);
  pinMode(trigPinb4, OUTPUT);
  pinMode(echoPinb4, INPUT);
  pinMode(trigPinb5, OUTPUT);
  pinMode(echoPinb5, INPUT);
  pinMode(trigPinb6, OUTPUT);
  pinMode(echoPinb6, INPUT);
  Serial.begin(115200);
  myservo.attach(4);  
  myservo.write(122);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }
  //Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}


void loop()
{
  float distanceR = 0;
  float distanceL =  0;
  distancef = calcdisf();
  Serial.print(distancef);
  Serial.println(" cm (f)");
 // analogWrite(ENA, spd);
 // analogWrite(ENB, spd);
  if(corner<3)
  {
    forward();
    //mop();
  }
  else
  {
    stp();
    //stop_mop();
  }

  // Create an array to hold distances from all six sonars
  float distances[6];

  // Calculate the distance for each sonar sensor
  distances[0] = calcdisb(trigPinb1, echoPinb1);
  distances[1] = calcdisb(trigPinb2, echoPinb2);
  distances[2] = calcdisb(trigPinb3, echoPinb3);
  distances[3] = calcdisb(trigPinb4, echoPinb4);
  distances[4] = calcdisb(trigPinb5, echoPinb5);
  distances[5] = calcdisb(trigPinb6, echoPinb6);
  // Check if any ground sonar detects an object closer than 30 cm
  if (checkSonarArray30(distances)) {
    stp();
    delay(100);
    if (distancef > 40)
    {
      //move forward ektu
      forward();  // Start moving forward

      while (true) {
        // Check if any sonar detects an object closer than 10 cm
        distances[0] = calcdisb(trigPinb1, echoPinb1);
        distances[1] = calcdisb(trigPinb2, echoPinb2);
        distances[2] = calcdisb(trigPinb3, echoPinb3);
        distances[3] = calcdisb(trigPinb4, echoPinb4);
        distances[4] = calcdisb(trigPinb5, echoPinb5);
        distances[5] = calcdisb(trigPinb6, echoPinb6);
        if (checkSonarArray10(distances)) {
          stp();  // Stop the car
          Serial.println("Stopped at 10 cm");
          break;  // Exit the loop and stop moving
        }
      }
      // Declare an array to hold active sonar indices
      int activeSonars[6];
      
      // Get the indices of sonars that have distance < 10 cm
      getSonarsBelowThreshold(distances, activeSonars);
      
      // Print the sonar indices with distance below threshold
      Serial.print("Sonars below threshold: ");
      for (int i = 0; activeSonars[i] != -1; i++) {
        Serial.print(activeSonars[i]);
        Serial.print(" ");
      }
      Serial.println()
      //send to python with serial coimmunication

      //arm er code 
    }
  else
  {
    stp();
    delay(100);
    distancel = calcdisl();
    distancer = calcdisr();
    if((distancel<30)||(distancer<30))
    {
      corner=corner+1;
    }
    distanceR=lookRight();
    delay(200);
    distanceL = lookLeft();
    Serial.print(distanceR);
    Serial.println(" cm (R)");
    Serial.print(distanceL);
    Serial.println(" cm (L)");
    
    delay(200);
    if((distanceR<100)&&(distanceL<100))
    {
      if(corner<3)
      {
        zigzag();
      }
      else
      {
        stp();
      }
    }
    else
    {
      z_angle=0;
      left();
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }
      unsigned long start_time = millis();
      forward();
      delay(100);
      //edit
      while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }
      unsigned long real_time = millis();
      //delay(200);
      z_angle=0;
      right();
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }
      forward();
      delay(500);
     while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }
      //delay(500);
      z_angle=0;
      right();
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }
      //forward();
      //delay(200);
      forward();
      delay(real_time - start_time);
//      while(calcdisr()<20)
//      {
//        forward();
//        if(calcdisr()>20)
//          break;
//      }
      z_angle=0;
      left();
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }
      forward();
    }
  }
  
}
