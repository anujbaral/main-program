#include <Servo.h>

const int trigPin = 13;  // Ultrasonic sensor Trigger Pin
const int echoPin = 12;  // Ultrasonic sensor Echo Pin
const int servoPin = 11; 

// Motor control pins: L298N H-bridge
const int enAPin = 6;   // Left motor PWM speed control
const int in1Pin = 7;   // Left motor Direction 1
const int in2Pin = 5;   // Left motor Direction 2
const int enBPin = 3;   // Right motor PWM speed control
const int in3Pin = 4;   // Right motor Direction 1
const int in4Pin = 2;   // Right motor Direction 2

Servo servo;

// Function declarations
void moveForward();
void moveBackward(int duration);
void turnRight();
void turnLeft();
void stopMotors();
int scanRight();
int scanLeft();
unsigned int measureDistance();
void returnToCenter();

void setup() {
    Serial.begin(9600);
  
    // Set pin modes
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(enAPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enBPin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
  
    servo.attach(servoPin);
    servo.write(90); // Center the servo
    delay(500);
    moveForward(); // Start moving forward
}

void loop() {
    int distance = measureDistance();
    Serial.println("Distance: " + String(distance) + " cm");

    if (distance < 20) { // Adjusted distance for early detection
        Serial.println("Obstacle detected! Stopping car...");
        stopMotors();
        delay(300);
        
        int distanceRight = scanRight();
        int distanceLeft = scanLeft();
        Serial.println("Distance Left: " + String(distanceLeft) + " cm, Distance Right: " + String(distanceRight) + " cm");
        
        returnToCenter();

        // Choose direction based on distance
        if (distanceRight > distanceLeft && distanceRight >= 30) {
            Serial.println("Turning right...");
            turnRight();
        } else if (distanceLeft > distanceRight && distanceLeft >= 30) {
            Serial.println("Turning left...");
            turnLeft();
        } else {
            Serial.println("Reversing slightly and turning...");
            moveBackward(200); // Reverse slightly
            delay(300);
            if (distanceLeft < distanceRight) {
                turnLeft(); // Turn left to avoid obstacle
            } else {
                turnRight(); // Turn right to avoid obstacle
            }
        }
    } else {
        Serial.println("Moving forward...");
        moveForward();
    }

    delay(100); // Stabilization delay
}

int scanRight() {
    servo.write(30); // Turn servo right
    delay(200); // Allow time for the servo to stabilize
    return measureDistance();
}

int scanLeft() {
    servo.write(150); // Turn servo left
    delay(200); // Allow time for the servo to stabilize
    return measureDistance();
}

unsigned int measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.0343 / 2; // Convert to centimeters
}

void returnToCenter() {
    servo.write(90); // Return servo to center position
    delay(300); // Shortened delay for quicker recovery
}

void moveForward() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW); // Left motor forward
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW); // Right motor forward
    analogWrite(enAPin, 255);
    analogWrite(enBPin, 255);
}

void moveBackward(int duration) {
    digitalWrite(in1Pin, LOW);  // Left motor backward
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, LOW);  // Right motor backward
    digitalWrite(in4Pin, HIGH);
    
    analogWrite(enAPin, 255);
    analogWrite(enBPin, 255);

    delay(duration); // Reverse for a short duration
    stopMotors();
}

void turnRight() {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW); // Stop left motor
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW); // Right motor forward
    analogWrite(enBPin, 255); // Full speed right motor

    delay(500); // Adjust turn duration
    stopMotors();
}

void turnLeft() {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW); // Left motor forward
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW); // Stop right motor
    analogWrite(enAPin, 255); // Full speed left motor

    delay(500); // Adjust turn duration
    stopMotors();
}

void stopMotors() {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
    analogWrite(enAPin, 0);
    analogWrite(enBPin, 0);
}
