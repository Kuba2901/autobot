// Pin definitions for L298N Motor Driver
const int STEERING_ENA = 9;  // PWM pin for steering motor speed
const int STEERING_IN1 = 8;  // Direction pin 1 for steering motor
const int STEERING_IN2 = 7;  // Direction pin 2 for steering motor

const int TRACTION_ENB = 3;  // PWM pin for traction motor speed
const int TRACTION_IN3 = 5;  // Direction pin 1 for traction motor
const int TRACTION_IN4 = 4;  // Direction pin 2 for traction motor

// Buffer for incoming serial data
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);

  // Configure motor control pins
  pinMode(STEERING_ENA, OUTPUT);
  pinMode(STEERING_IN1, OUTPUT);
  pinMode(STEERING_IN2, OUTPUT);
  pinMode(TRACTION_ENB, OUTPUT);
  pinMode(TRACTION_IN3, OUTPUT);
  pinMode(TRACTION_IN4, OUTPUT);

  // Ensure motors are stopped at startup
  stopMotors();
}

void loop() {
  // Read serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Process complete command when newline is received
    if (c == '\n') {
      buffer[bufferIndex] = '\0';  // Null terminate the string
      processCommand(buffer);
      bufferIndex = 0;  // Reset buffer
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;
    }
  }
}

void processCommand(char* cmd) {
  // Handle initialization command
  if (strcmp(cmd, "INIT") == 0) {
    stopMotors();
    Serial.println("OK");
    return;
  }
  
  // Handle stop command
  if (strcmp(cmd, "STOP") == 0) {
    stopMotors();
    Serial.println("OK");
    return;
  }

  // Handle motor control command
  if (cmd[0] != 'M') {
    Serial.println("ERROR: Invalid Command");
    return;
  }

  char* token;
  int steering_pwm, traction_pwm, forward;

  token = strtok(cmd + 2, ",");
  if (token == NULL || (steering_pwm = atoi(token)) < 0 || steering_pwm > 255) {
    Serial.println("ERROR: Invalid Steering PWM");
    return;
  }

  token = strtok(NULL, ",");
  if (token == NULL || (traction_pwm = atoi(token)) < 0 || traction_pwm > 255) {
    Serial.println("ERROR: Invalid Traction PWM");
    return;
  }

  token = strtok(NULL, ",");
  if (token == NULL || (forward = atoi(token)) < 0 || forward > 1) {
    Serial.println("ERROR: Invalid Direction");
    return;
  }

  setSteeringMotor(steering_pwm);
  setTractionMotor(traction_pwm, forward);
  Serial.println("OK");
}

void setSteeringMotor(int pwm) {
  pwm = constrain(pwm, 0, 255);  // Ensure PWM is within bounds

  if (pwm == 0) {
    // Stop the steering motor
    digitalWrite(STEERING_IN1, LOW);
    digitalWrite(STEERING_IN2, LOW);
    analogWrite(STEERING_ENA, 0);
    return;
  }

  if (pwm > 127) {
    digitalWrite(STEERING_IN1, HIGH);
    digitalWrite(STEERING_IN2, LOW);
    pwm = map(pwm, 128, 255, 0, 255);  // Normalize for forward motion
  } else {
    digitalWrite(STEERING_IN1, LOW);
    digitalWrite(STEERING_IN2, HIGH);
    pwm = map(pwm, 127, 0, 0, 255);  // Normalize for reverse motion
  }

  analogWrite(STEERING_ENA, pwm);
}

void setTractionMotor(int pwm, int forward) {
  pwm = constrain(abs(pwm), 0, 255);  // Ensure PWM is positive and within range

  if (pwm == 0) {
    // Ensure the motor is fully stopped
    digitalWrite(TRACTION_IN3, LOW);
    digitalWrite(TRACTION_IN4, LOW);
    analogWrite(TRACTION_ENB, 0);
    return;
  }

  if (forward) {
    digitalWrite(TRACTION_IN3, HIGH);
    digitalWrite(TRACTION_IN4, LOW);
  } else {
    digitalWrite(TRACTION_IN3, LOW);
    digitalWrite(TRACTION_IN4, HIGH);
  }

  analogWrite(TRACTION_ENB, pwm);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(STEERING_IN1, LOW);
  digitalWrite(STEERING_IN2, LOW);
  digitalWrite(TRACTION_IN3, LOW);
  digitalWrite(TRACTION_IN4, LOW);
  analogWrite(STEERING_ENA, 0);
  analogWrite(TRACTION_ENB, 0);
}