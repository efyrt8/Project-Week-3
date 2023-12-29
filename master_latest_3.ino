// Master code
#include <Wire.h>
#include <MPU6050.h>
// 5 components (Motor, IR sensor, LCD, encoder sensor, mpu-6050)
// 2 components in slave board (LCD and encoder sensor)
// 3 components at master board (IR sensor, motor, MPU-6050)

// Motor A connections
int enA = 11;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A2;
int in4 = A3;

// MPU-6050
MPU6050 mpu;
int16_t accX, accY, accZ;
int16_t x, y, z;

// IR sensor
int LS = 13;
int RS = 12;

// Use switch case for different stage of task
// 1 - go up the ramp and display the angle
// 2 - stop on top of the ramp and spin for 1 round
// 3 - go down the ramp and stop
// 4 - continue the track for 70cm (Group number is 7) then stop for 2 seconds
// 5 - continue the path until the end and display distance convered and time
int stage = 1;

// angle variable
int16_t angle;
int MAX_ANGLE = 0;

// distance variable
int distance;

void setup() {
  Wire.begin(9); // Set the slave address to 9
  Serial.begin(9600);
  mpu.initialize();

  // Send signals of MPU-6050 to master board when requested
  Wire.onRequest(requestAngle);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
    
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // IR sensors control
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);

  Wire.onReceive(receiveDistance);

  delay(5000);
}

void receiveDistance(){
  distance = Wire.read();
}

void moveForward(){
  analogWrite(enA,60);
  analogWrite(enB,60);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveForward_begin(){
  analogWrite(enA,75);
  analogWrite(enB,75);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void headstart(){
  analogWrite(enA,75);
  analogWrite(enB,75);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveLeft(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveRight(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveLeft_slant(){
  analogWrite(enA,170);
  analogWrite(enB,170);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveRight_slant(){
  analogWrite(enA,170);
  analogWrite(enB,170);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void stopMoving(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void spin_around(){
  while (yawAngle<=360){
    analogWrite(enA,120);
    analogWrite(enB,120);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
    stopMoving();
    delay(2000);
}

void speedUp(){
  analogWrite(enA,135);
  analogWrite(enB,135);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void speedUp_plus(){
  analogWrite(enA,180);
  analogWrite(enB,180);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void readAngle() {
  // Read raw gyro data
  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Convert raw gyro data to degrees per second (adjust sensitivity as needed)
  float gyroX_degPerSec = gyroX / 131.0; // Sensitivity for 250°/s
  float gyroY_degPerSec = gyroY / 131.0;
  float gyroZ_degPerSec = gyroZ / 131.0;

  // Calculate pitch and yaw angles based on gyroscopic data
  float deltaTime = 0.01; // Data sampled every 0.01s
  float angle = gyroY_degPerSec * deltaTime; // Pitch angle change in degrees
  float yawAngle = gyroZ_degPerSec * deltaTime;   // Yaw angle change in degrees

    // Keep yaw angle within the range 0 to 360 degrees
    if (yawAngle < 0) {
      yawAngle += 360;
    }

    if (yawAngle >360){
      yawAngle -= 360;
    }

    // Update MAX_ANGLE conditionally if necessary (you may adjust this condition based on your requirements)
    if ((angle > 0) && (angle > MAX_ANGLE) && (angle < 90)) {
        MAX_ANGLE = angle;
    }
}


void headStart_following(){
  if ((digitalRead(LS) == HIGH) && (digitalRead(RS) == HIGH))  headstart(); // Send signal to move forward
  else if ((digitalRead(LS)==LOW) && (digitalRead(RS) == HIGH)) moveLeft(); // Send signal to turn left
  else if ((digitalRead(LS) == HIGH) && (digitalRead(RS)==LOW)) moveRight(); // Send signal to turn right
  else stopMoving(); // Send signal to stop
}

void line_following(){
  if ((digitalRead(LS) == HIGH) && (digitalRead(RS) == HIGH))  moveForward(); // Send signal to move forward
  else if ((digitalRead(LS)==LOW) && (digitalRead(RS) == HIGH)) moveLeft(); // Send signal to turn left
  else if ((digitalRead(LS) == HIGH) && (digitalRead(RS)==LOW)) moveRight(); // Send signal to turn right
  else stopMoving(); // Send signal to stop
}

void requestAngle() {
  // Send angles to the master
  Wire.write((byte)(angle >> 8));
  Wire.write((byte)(angle & 0xFF));
}

void linefollowing_minus(){
  if ((digitalRead(LS) == HIGH) && (digitalRead(RS) == HIGH))  moveForward(); // Accelerate up hill
  else if ((digitalRead(LS)==LOW) && (digitalRead(RS) == HIGH)) moveLeft_slant(); // Send signal to turn left
  else if ((digitalRead(LS) == HIGH) && (digitalRead(RS)==LOW)) moveRight_slant(); // Send signal to turn right
  else stopMoving(); // Send signal to stopline_following();
}

void loop() {
  // Calculate the tilt angle
    switch (stage){
    case 1:
    // 1 - go up the ramp and display the angle
    // Read angle from slave board
    // display angle on lcd
    // if angle more than 5 degree, enter stage 2
    Serial.println("Successfully enter stage 1 (Move from the ground)");

    readAngle();
    Serial.print("Angle: ");
    Serial.println(angle);

    if (MAX_ANGLE<11 || angle>340){
      line_following();
    } else {
      stage = 2;
      break;
    }
  
    break;

    case 2:
    // 2 - stop on top of the ramp and spin for 1 round
    // once angle go back to less than 2 degree, car stops
    // then spin 1 round
    // stop again for a second
    // continue to go down the ramp
    // Enter stage 3
    // Without loop
    Serial.println("Sucessfully enter stage 2 (Going up ramp)");

    readAngle();
    Serial.print("Stage 2: ");
    Serial.println(angle);
    if (angle>3){
    if ((digitalRead(LS) == HIGH) && (digitalRead(RS) == HIGH))  speedUp(); // Accelerate up hill
    else if ((digitalRead(LS)==LOW) && (digitalRead(RS) == HIGH)) moveLeft_slant(); // Send signal to turn left
    else if ((digitalRead(LS) == HIGH) && (digitalRead(RS)==LOW)) moveRight_slant(); // Send signal to turn right
    else speedUp(); // Send signal to stop
    } else {
      // Stop the car

      stopMoving();
      delay(1000);
      // spin for 1 round and stop
      stage = 21;
    }

    break;

    case 21:
    // 21 - double check
    Serial.println("Sucessfully enter stage 21 (Double check)");

    readAngle();
    Serial.print("Stage 21: ");
    Serial.println(angle);
    if ((angle>=2) && (angle<=50)){
      stage = 2;
      break;
    } else {
      // Stop the car
      /*
      speedUp();
      delay(200);
      */
      stopMoving();
      delay(400);
      Serial.println("Spining 1 round");
      delay(4000);
      spin_around();
      // spin for 1 round and stop
      stage = 3;
    }

    break;


    case 3:
    // 3 - continue to follow track on top of the ramp
    // enter stage 4 when going down the ramp
    
    Serial.println("Successfully enter stage 3 (On the ramp)");

    readAngle();

    if ((angle<=50) || (angle>330)) {
      if ((digitalRead(LS) == HIGH) && (digitalRead(RS) == HIGH))  moveForward(); // Accelerate up hill
      else if ((digitalRead(LS)==LOW) && (digitalRead(RS) == HIGH)) moveLeft_slant(); // Send signal to turn left
      else if ((digitalRead(LS) == HIGH) && (digitalRead(RS)==LOW)) moveRight_slant(); // Send signal to turn right
      else moveForward(); // Send signal to continue moving
      Serial.print("On the ramp: ");
      Serial.println(angle);
    } else {
      Serial.print("Going into stage 4: ");
      Serial.println(angle);
      stage = 4;
      break;
    }
    break;

    case 4:
    // 4 - going down the ramp by following the track
    // enter stage 5 when on ground

    Serial.println("Successfully enter stage 4 (Going down the ramp & stop after ramp)");

    readAngle();

    if (angle>=300){
    headstart();
    Serial.print("Going down the ramp: ");
    Serial.println(angle);    
    } else {
      stopMoving();
      delay(2000); // Stop for 2 seconds after going down the ramp
      stage = 5;
      break;
    }
    break;

    case 5:
    // 5 - follow line for 70 cm

    Serial.println("Successfully enter stage 5 (Follow the track for 70cm)");

    if (distance <=70){
      headStart_following();
    } else{
      stopMoving();
      delay(2000); // stop for 2 seconds after moving for 70 cm
      stage = 6;
      break;
    }
    Serial.print("Distance: ");
    Serial.println(distance);

    break;

    case 6:

    Serial.println("Successfully enter stage 6 (Continue to follow the track)");   

    line_following();
    
    break;

    }
}


