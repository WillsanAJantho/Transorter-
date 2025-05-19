#include <Arduino.h>
#include <math.h>
#include <Servo.h>

const float z_offset = 7.5;
const int zero_point_T3 = 235;

const float L1 = 12.0; // Length of the first link
const float L2 = 16.0; // Length of the second link

// Servo objects for controlling the servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo SerMag; // Servo for magnet control

// Function to move a servo slowly to a target angle
void move_servo_slow(Servo &servo, int target_angle) {
  int current_angle = servo.read();
  if (current_angle < target_angle) {
    for (int angle = current_angle; angle <= target_angle; angle++) {
      servo.write(angle);
      delay(10);
    }
  } else {
    for (int angle = current_angle; angle >= target_angle; angle--) {
      servo.write(angle);
      delay(10);
    }
  }
}

// Function to calculate inverse kinematics for a 2-link planar robot
void calculate_thetas(float x, float z, float &theta1, float &theta2) {
  float distance = sqrt(x * x + z * z);

  if (distance > (L1 + L2) || distance < fabs(L1 - L2)) {
    Serial.println("Point is out of reach for the given link lengths.");
    return;
  }

  float alpha = acos((L1 * L1 + distance * distance - L2 * L2) / (2 * L1 * distance));
  float beta = acos((L1 * L1 + L2 * L2 - distance * distance) / (2 * L1 * L2));
  float phi = atan2(z, x);

  theta1 = phi + alpha;  // Elbow-down configuration
  theta2 = M_PI - beta;  // Angle at the second joint

  theta1 = degrees(theta1);
  theta2 = degrees(theta2);
}

float calculate_theta3(float x, float z) {
  // First, calculate the base angle based on x and z
  float theta3_rad = atan2(x, z);  // atan2 gives the angle in radians

  // Convert it to degrees
  float theta3_deg = degrees(theta3_rad);

  // Handle dead zone and adjust based on the conditions
  if (theta3_deg > 360) {
    theta3_deg -= 360;  // Keep theta3 within a 360-degree range
  } else if (theta3_deg < -330) {
    theta3_deg += 360;  // Adjust angle to avoid falling in the dead zone
  }

  // Apply conditions for different regions based on the signs of x and z
  if (x > 0 && z > 0) {
    // Quadrant 1: theta3 should be between -330 and -235
    theta3_deg = constrain(theta3_deg, -330, -235);
  } else if (x < 0 && z > 0) {
    // Quadrant 2: theta3 should be between -235 and -145
    theta3_deg = constrain(theta3_deg, -235, -145);
  } else if (x < 0 && z < 0) {
    // Quadrant 3: theta3 should be between -145 and -55
    theta3_deg = constrain(theta3_deg, -145, -55);
  } else if (x > 0 && z < 0) {
    // Quadrant 4: theta3 should be between -55 and 0
    theta3_deg = constrain(theta3_deg, -55, 0);
  }

  // Ensure that theta3 stays within the defined range
  return theta3_deg;
}


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // Wait for the serial port to connect.
  }

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  SerMag.attach(12); // Attach the magnet servo

  servo1.write(130);
  servo2.write(90);

  SerMag.write(20);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int comma = input.indexOf(',');

    if (comma > 0) {
      String x_str = input.substring(0, comma);
      String z_str = input.substring(comma + 1);

      float x = x_str.toFloat();
      float z = z_str.toFloat();

      z += z_offset;
      float target_y = 3;
      float distance = sqrt(x * x + z * z);

      for (float current_y = 18; current_y >= target_y; current_y -= 3) {
        float theta1, theta2;
        calculate_thetas(distance, current_y, theta1, theta2);
        float theta3 = calculate_theta3(x, z);

        if (theta3 == -999) {
          return;
        }

        Serial.print("For y = ");
        Serial.print(current_y);
        Serial.print(", Calculated angles - Theta 1: ");
        Serial.print(theta1, 2);
        Serial.print("\xC2\xB0, Theta 2: ");
        Serial.print(theta2, 2);
        Serial.print("\xC2\xB0, Theta 3: ");
        Serial.println(theta3, 2);

        int servo1_angle = constrain(map(theta1, 0, 180, 0, 180), 0, 180);
        int servo2_angle = constrain(map(theta2, 0, 180, 0, 180), 0, 180);
        int servo3_angle = constrain(map(theta3, -235, 235, 0, 180), 0, 180);

        servo1.write(servo1_angle);
        servo2.write(servo2_angle);
        servo3.write(servo3_angle);

        Serial.println("Press Enter to continue.");
        while (Serial.available() == 0) {
          // Wait for user input
        }
        Serial.read(); // Clear the input buffer
      }

      // pencet tombol
      SerMag.write(6);
      delay(500);
      // lepas tombol
      SerMag.write(20);
      delay(500);

      for (float current_y = target_y; current_y <= 18; current_y += 3) {
        float theta1, theta2;
        calculate_thetas(distance, current_y, theta1, theta2);
        float theta3 = calculate_theta3(x, z);

        if (theta3 == -999) {
          return;
        }

        Serial.print("For y = ");
        Serial.print(current_y);
        Serial.print(", Calculated angles - Theta 1: ");
        Serial.print(theta1, 2);
        Serial.print("\xC2\xB0, Theta 2: ");
        Serial.print(theta2, 2);
        Serial.print("\xC2\xB0, Theta 3: ");
        Serial.println(theta3, 2);

        int servo1_angle = constrain(map(theta1, 0, 180, 0, 180), 0, 180);
        int servo2_angle = constrain(map(theta2, 0, 180, 0, 180), 0, 180);
        int servo3_angle = constrain(map(theta3, -235, 235, 0, 180), 0, 180);

        servo1.write(servo1_angle);
        servo2.write(servo2_angle);
        //servo3.write(servo3_angle);

        Serial.println("Press Enter to continue.");
        while (Serial.available() == 0) {
          // Wait for user input
        }
        Serial.read(); // Clear the input buffer
      }

      // After completing everything, reset the servo angles slowly
      move_servo_slow(servo1, 130); // Move Servo1 to 130 slowly
      move_servo_slow(servo2, 90);  // Move Servo2 to 90 slowly
      delay(2000); // Pause for 2 seconds
      Serial.println("Servos reset to Theta 1 = 130, Theta 2 = 90.");
    } else {
      Serial.println("Invalid input format. Please enter in the format: x, z");
    }
  }

  delay(100);
}

