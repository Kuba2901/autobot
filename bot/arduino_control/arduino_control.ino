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
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure L298N control pins
  pinMode(STEERING_ENA, OUTPUT);
  pinMode(STEERING_IN1, OUTPUT);
  pinMode(STEERING_IN2, OUTPUT);
  pinMode(TRACTION_ENB, OUTPUT);
  pinMode(TRACTION_IN3, OUTPUT);
  pinMode(TRACTION_IN4, OUTPUT);
  
  // Initialize all motors to stopped state
  digitalWrite(STEERING_IN1, LOW);
  digitalWrite(STEERING_IN2, LOW);
  digitalWrite(TRACTION_IN3, LOW);
  digitalWrite(TRACTION_IN4, LOW);
  analogWrite(STEERING_ENA, 0);
  analogWrite(TRACTION_ENB, 0);
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
  // Check for initialization command
  if (strcmp(cmd, "INIT") == 0) {
    stopMotors();
    Serial.println("OK");
    return;
  }
  
  // Check for stop command
  if (strcmp(cmd, "STOP") == 0) {
    stopMotors();
    Serial.println("OK");
    return;
  }
  
  // Process motor command: "M,<steering_pwm>,<traction_pwm>,<direction>"
  if (cmd[0] == 'M') {
    int steering_pwm, traction_pwm;
    int forward;
    
    // Parse the command
    char* token = strtok(cmd + 2, ",");  // Skip "M,"
    if (token != NULL) {
      steering_pwm = atoi(token);
      
      token = strtok(NULL, ",");
      if (token != NULL) {
        traction_pwm = atoi(token);
        
        token = strtok(NULL, ",");
        if (token != NULL) {
          forward = atoi(token);
          
          // Set motor speeds and directions
          setSteeringMotor(steering_pwm);
          setTractionMotor(traction_pwm);
          
          Serial.println("OK");
          return;
        }
      }
    }
  }
  
  // If we get here, the command was invalid
  Serial.println("ERROR");
}

void turnRight()
{
    digitalWrite(STEERING_IN1, HIGH);
    digitalWrite(STEERING_IN2, LOW);
}

void turnLeft()
{
    digitalWrite(STEERING_IN1, LOW);
    digitalWrite(STEERING_IN2, HIGH);
}

void setSteeringMotor(int pwm) {
  // Determine direction based on PWM value
  if (!pwm)
    return ;
  if (pwm > 127)
    turnRight();
  else
    turnLeft();
  
  // Set speed
  analogWrite(STEERING_ENA, 255);
}

void setTractionMotor(int pwm) {
  // Set direction
  int forward;

  if (!pwm)
    return ;
  forward = pwm < 127 ? 1: 0;
  Serial.print(pwm);
  Serial.print(" ");
  Serial.print(forward);
  Serial.print('\n');
  if (forward) {
    digitalWrite(TRACTION_IN3, HIGH);
    digitalWrite(TRACTION_IN4, LOW);
  } else {
    digitalWrite(TRACTION_IN3, LOW);
    digitalWrite(TRACTION_IN4, HIGH);
  }
  
  // Set speed
  analogWrite(TRACTION_ENB, 255);
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