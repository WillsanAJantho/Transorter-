#include <Servo.h>

#define outputA 2
#define outputB 3

Servo myServo;
const int limitSwitchPin = 4;

Servo servo1;
Servo servo2;
int counter = 0;
int aState;
int aLastState;
float angle_per_pulse = 2.25; // Encoder step size
float current_angle = 0;
float desired_angle = 0;
unsigned long lastMovementTime = 0;
const unsigned long timeout = 14000;

bool angleSet = false;

// PWM pulse widths for the servo (in microseconds)
const int servoStop = 1500;    // Neutral
const int servoForward = 1630; // Clockwise
const int servoBackward = 1360; // Counterclockwise

const float deadZone = 3.0; // Dead zone size in degrees (±3°)

void setup() {
    pinMode(outputA, INPUT);
    pinMode(outputB, INPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);

    myServo.attach(9); // Attach servo to pin 9
    myServo.writeMicroseconds(servoStop); // Initialize servo to stop

    Serial.begin(9600);
    Serial.println("Enter the desired angle of rotation to begin:");
    aLastState = digitalRead(outputA);

    // Find limit switch with timeout
    unsigned long startTime = millis();
    while (digitalRead(limitSwitchPin) == HIGH && millis() - startTime < 10000) {
        myServo.writeMicroseconds(servoForward); // Move forward to find the limit switch
        delay(10);
    }
    myServo.writeMicroseconds(servoStop); // Stop servo after finding limit
}

void loop() {
    if (!angleSet) {
        if (Serial.available() > 0) {

        // Find limit switch with timeout
        unsigned long startTime = millis();
        while (digitalRead(limitSwitchPin) == HIGH && millis() - startTime < 10000) {
            myServo.writeMicroseconds(servoForward); // Move forward to find the limit switch
            delay(10);
        }
        myServo.writeMicroseconds(servoStop); // Stop servo after finding limit
        counter = 0;

          Serial.println("A");
            desired_angle = Serial.parseFloat();
            Serial.print("Desired Angle Set To: ");
            Serial.println(desired_angle);
            angleSet = true;
            lastMovementTime = millis();
        }
        return;
    }
Serial.println(desired_angle);
    // Encoder logic without updating lastMovementTime
    aState = digitalRead(outputA);
    if (aState != aLastState) {
        if (digitalRead(outputB) != aState) {
            counter++;
        } else {
            counter--;
        }

        current_angle = counter * angle_per_pulse;
        Serial.print("Current Angle: ");
        Serial.println(current_angle);
    }
    aLastState = aState;

    // Calculate the error
    float error = desired_angle - current_angle;

    // Adjust servo for constant speed
    if (abs(error) > deadZone) { // Only move if outside the dead zone
        if (error > 0) {
            // Move forward (clockwise)
            myServo.writeMicroseconds(servoForward);
        } else {
            // Move backward (counterclockwise)
            myServo.writeMicroseconds(servoBackward);
        }
    } else {
        // Stop the servo if within the dead zone
        myServo.writeMicroseconds(servoStop);
        angleSet = false; // Allow resetting desired angle
        Serial.read();
        
    }

    // Timeout logic
    // if (millis() - lastMovementTime >= timeout) {
    //     float remaining_angle = desired_angle - current_angle;
    //     Serial.println("\nNo movement detected for 7 seconds.");
    //     Serial.print("Remaining Angle to Desired Position: ");
    //     Serial.println(remaining_angle);
    //     myServo.writeMicroseconds(servoStop);
    //     angleSet = false; // Allow resetting desired angle
    //     Serial.read(); // clear buffer

    // }
}
