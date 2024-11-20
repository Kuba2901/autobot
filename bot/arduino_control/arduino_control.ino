// Pin definitions for L298N motor driver
const int STEERING_ENA = 9;  // PWM pin for steering motor speed
const int STEERING_IN1 = 2;  // Direction pin 1 for steering motor
const int STEERING_IN2 = 3;  // Direction pin 2 for steering motor

const int TRACTION_ENB = 10;  // PWM pin for traction motor speed
const int TRACTION_IN3 = 4;  // Direction pin 1 for traction motor
const int TRACTION_IN4 = 5;  // Direction pin 2 for traction motor

// Serial communication
const int BUFFER_SIZE = 32;
char buffer[BUFFER_SIZE];
int buffer_index = 0;

// TRACTION VALUES
const int TRAC_NEUTRAL = 127;

// STEERING VALUES
const int STEER_NEUTRAL = 127;

// Minimal PWM on the motors
const int MIN_MOTOR_THRESHOLD = 160;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // Match the baud rate in ROS2 code
  
  // Configure motor control pins
  pinMode(STEERING_ENA, OUTPUT);
  pinMode(STEERING_IN1, OUTPUT);
  pinMode(STEERING_IN2, OUTPUT);
  pinMode(TRACTION_ENB, OUTPUT);
  pinMode(TRACTION_IN3, OUTPUT);
  pinMode(TRACTION_IN4, OUTPUT);
  
  // Initialize motors to stopped state
  stopMotors();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    // Add character to buffer if not end of command
    if (c != '\n' && buffer_index < BUFFER_SIZE - 1) {
      buffer[buffer_index++] = c;
    } else {
      // Null terminate the string
      buffer[buffer_index] = '\0';
      
      // Process the command
      processCommand(buffer);
      
      // Reset buffer
      buffer_index = 0;
      
      // Send acknowledgment
      Serial.println("OK");
    }
  }
}

void processCommand(const char* cmd) {
  if (strncmp(cmd, "INIT", 4) == 0) {
    stopMotors();
  }
  else if (strncmp(cmd, "STOP", 4) == 0) {
    stopMotors();
  }
  else if (cmd[0] == 'M') {
    // Parse motor command: M,<steering_angle>,<traction_pwm>
    int steering_pwm = 0;
    int traction_pwm = 0;
    
    char* ptr = strchr(cmd, ',');
    if (ptr) {
      steering_pwm = atoi(ptr + 1);
      ptr = strchr(ptr + 1, ',');
      if (ptr) {
        traction_pwm = atoi(ptr + 1);
      }
    }
    
    // Apply commands to motors
    setSteeringMotor(steering_pwm);
    setTractionMotor(traction_pwm);
  }
}

void setSteeringMotor(int pwm) {
  Serial.print("Steer PWM: ");
  Serial.print(pwm);
  Serial.print("\n");
  if (pwm == STEER_NEUTRAL)
  {
    Serial.println("Going straight");
    // GO STRAIGHT
    digitalWrite(STEERING_IN1, LOW);
    digitalWrite(STEERING_IN2, LOW);
    analogWrite(STEERING_ENA, 0);
    return ;
  }
  if (pwm < STEER_NEUTRAL) {
    // TURN RIGHT
    pwm = (STEER_NEUTRAL - pwm) * 2;
    digitalWrite(STEERING_IN1, HIGH);
    digitalWrite(STEERING_IN2, LOW);
    analogWrite(STEERING_ENA, pwm);
  } else {
    // TURN LEFT
    pwm = map(pwm, 128, 255, 0, 255);
    digitalWrite(STEERING_IN1, LOW);
    digitalWrite(STEERING_IN2, HIGH);
    analogWrite(STEERING_ENA, pwm);
  }
}

void setTractionMotor(int pwm) {
  Serial.print("Trac PWM: ");
  Serial.print(pwm);
  Serial.print("\n");
  if (pwm == TRAC_NEUTRAL)
  {
    // STOP THE MOTOR
    digitalWrite(TRACTION_IN3, LOW);
    digitalWrite(TRACTION_IN4, LOW);
    analogWrite(TRACTION_ENB, 0);
    return ;    
  }
  if (pwm < TRAC_NEUTRAL)
  {
    // GO REVERSE
    pwm = (STEER_NEUTRAL - pwm) * 2;
    digitalWrite(TRACTION_IN3, HIGH);
    digitalWrite(TRACTION_IN4, LOW);
    analogWrite(TRACTION_ENB, pwm);
  }
  else
  {
    // GO FORWARD
    pwm = map(pwm, 128, 255, 0, 255);
    digitalWrite(TRACTION_IN3, LOW);
    digitalWrite(TRACTION_IN4, HIGH);
    analogWrite(TRACTION_ENB, pwm);
  }
}

void stopMotors() {
  // Stop steering motor
  digitalWrite(STEERING_IN1, LOW);
  digitalWrite(STEERING_IN2, LOW);
  analogWrite(STEERING_ENA, 0);
  
  // Stop traction motor
  digitalWrite(TRACTION_IN3, LOW);
  digitalWrite(TRACTION_IN4, LOW);
  analogWrite(TRACTION_ENB, 0);
}