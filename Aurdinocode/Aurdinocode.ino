#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);  // Attach servo to pin 9
  servo2.attach(10); // Attach servo to pin 10
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the incoming string until newline character

    // Find the position of ":" character in the string
    int splitIndex = input.indexOf(':');

    // If ":" is found
    if (splitIndex != -1) {
      // Extract the first integer before ":"
      int value1 = input.substring(0, splitIndex).toInt();

      // Extract the second integer after ":"
      int value2 = input.substring(splitIndex + 1).toInt();

      // Move servo1 to the first integer value
      servo1.write(value1);

      // Move servo2 to the second integer value
      servo2.write(value2);
      Serial.println("OK");
    }
  }
  Serial.println("OK");
}
