typedef enum  s_robot_state
{
  ERROR,
  OFF,
  INITIALIZED,
  ON,
  REVERSING
} t_robot_state;

typedef struct  s_init_lights
{
  int pin;
  int brightness;
  int fade_amount;
} t_init_lights;

typedef struct  s_config
{
  long int      baud_rate;
  int           steering_ena, steering_in1, steering_in2;
  int           traction_enb, traction_in3, traction_in4;
  int           headlights_pin, taillights_pin;
  char          buffer[32];
  int           buffer_index, buffer_size;
  int           pwm_neutral_val;
  int           failure_on_startup;
  t_init_lights init_lights;
  t_robot_state robot_state;
} t_config;

t_config  conf;

void  _setup_conf(
  long int    baud_rate,
  int         steering_ena,
  int         steering_in1,
  int         steering_in2,
  int         traction_enb,
  int         traction_in3,
  int         traction_in4,
  int         headlights_pin,
  int         taillights_pin,
  int         initlights_pin,
  int         pwm_neutral_val,
  int         buffer_size
)
{
  conf.baud_rate = baud_rate;
  conf.steering_ena = steering_ena;
  conf.steering_in1 = steering_in1;
  conf.steering_in2 = steering_in2;
  conf.traction_enb = traction_enb;
  conf.traction_in3 = traction_in3;
  conf.traction_in4 = traction_in4;
  conf.headlights_pin = headlights_pin;
  conf.taillights_pin = taillights_pin;
  conf.buffer_index = 0;
  conf.pwm_neutral_val = pwm_neutral_val;
  conf.failure_on_startup = 0;
  conf.robot_state = OFF;
  conf.buffer_size = buffer_size;
  conf.taillights_pin = initlights_pin;

  // Init lights
  t_init_lights init_lights;
  init_lights.brightness = 0;
  init_lights.fade_amount = 5;
  init_lights.pin = initlights_pin;
  conf.init_lights = init_lights;

  pinMode(conf.steering_ena, OUTPUT);
  pinMode(conf.steering_in1, OUTPUT);
  pinMode(conf.steering_in2, OUTPUT);
  pinMode(conf.traction_enb, OUTPUT);
  pinMode(conf.traction_in3, OUTPUT);
  pinMode(conf.traction_in4, OUTPUT);
  pinMode(conf.headlights_pin, OUTPUT);
  pinMode(conf.taillights_pin, OUTPUT);
  pinMode(conf.init_lights.pin, OUTPUT);
}


// ADD ERROR HANDLING WITH CORRECT ROBOT STATE
void setup() {
  _setup_conf(115200, 9, 2, 3, 10, 4, 5, 6, 7, 8, 127, 32);

  stopMotors();
  Serial.begin(conf.baud_rate);
  Serial.println("Setup Complete"); // Debug print
}


void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    // Add character to buffer if not end of command
    if (c != '\n' && conf.buffer_index < conf.buffer_size - 1) {
      conf.buffer[conf.buffer_index++] = c;
    } else {
      // Null terminate the string
      conf.buffer[conf.buffer_index] = '\0';
      
      // Process the command
      processCommand(conf.buffer);
      
      // Reset buffer
      conf.buffer_index = 0;
      
      // Send acknowledgment
      Serial.println("OK");
    }
  }
  switch (conf.robot_state)
  {
    case INITIALIZED:
      _enable_initialized_lights();
      break ;
    case ON:
      _enable_on_lights();
      break ;
    case OFF:
      _disable_lights();
    default:
      return ;
  }
}

void processCommand(const char* cmd) {
  if (strncmp(cmd, "INIT", 4) == 0) {
    enable_init_mode();
  }
  else if (strncmp(cmd, "START", 5) == 0)
    enable_on_mode();
  else if (!strncmp(cmd, "DISCONNECT", 10))
    disable_robot();
  else if (!strncmp(cmd, "OFF", 3) || !strncmp(cmd, "STOP", 4))
  {
    enable_off_mode();
    stopMotors();
  }
  else if (cmd[0] == 'M')
  {
    if (conf.robot_state != ON)
      Serial.println("The robot is not in on state!");
    else
      _parse_motor_command(cmd);
  }
  else
  {
    Serial.println("Unrecognized command.");
    stopMotors();
  }
}

void  _enable_initialized_lights(void)
{
  analogWrite(conf.init_lights.pin, conf.init_lights.brightness);
  analogWrite(conf.headlights_pin, conf.init_lights.brightness);

  // change the brightness for next time through the loop:
  conf.init_lights.brightness = conf.init_lights.brightness + conf.init_lights.fade_amount;

  // reverse the direction of the fading at the ends of the fade:
  if (conf.init_lights.brightness <= 0 || conf.init_lights.brightness >= 255) {
    conf.init_lights.fade_amount = -conf.init_lights.fade_amount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}

void  enable_init_mode(void)
{
  // stopMotors();
  conf.robot_state = INITIALIZED;
}

void  enable_off_mode(void)
{
  conf.robot_state = OFF;
  _disable_lights();
}

void  _disable_lights(void)
{
  digitalWrite(conf.init_lights.pin, LOW);
  digitalWrite(conf.headlights_pin, LOW);
  digitalWrite(conf.taillights_pin, LOW); 
}

void  disable_robot(void)
{
  _disable_lights();
  conf.robot_state = OFF;
  Serial.println("Disable request");
  Serial.end();
}


void  _enable_on_lights(void)
{
  digitalWrite(conf.init_lights.pin, LOW);
  digitalWrite(conf.taillights_pin, HIGH);
  digitalWrite(conf.headlights_pin, HIGH);
}

void  enable_on_mode(void)
{
  conf.robot_state = ON;
}

void  _parse_motor_command(const char *cmd)
{
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

void setSteeringMotor(int pwm) {
  Serial.print("Steer PWM: ");
  Serial.print(pwm);
  Serial.print("\n");
  if (pwm == conf.pwm_neutral_val)
  {
    Serial.println("Going straight");
    // GO STRAIGHT
    digitalWrite(conf.steering_in1, LOW);
    digitalWrite(conf.steering_in2, LOW);
    analogWrite(conf.steering_ena, 0);
    return ;
  }
  if (pwm < conf.pwm_neutral_val) {
    // TURN RIGHT
    pwm = (conf.pwm_neutral_val - pwm) * 2;
    digitalWrite(conf.steering_in1, HIGH);
    digitalWrite(conf.steering_in2, LOW);
    analogWrite(conf.steering_ena, pwm);
  } else {
    // TURN LEFT
    pwm = map(pwm, 128, 255, 0, 255);
    digitalWrite(conf.steering_in1, LOW);
    digitalWrite(conf.steering_in2, HIGH);
    analogWrite(conf.steering_ena, pwm);
  }
}

void setTractionMotor(int pwm) {
  Serial.print("Trac PWM: ");
  Serial.print(pwm);
  Serial.print("\n");
  if (pwm == conf.pwm_neutral_val)
  {
    // STOP THE MOTOR
    digitalWrite(conf.traction_in3, LOW);
    digitalWrite(conf.traction_in4, LOW);
    analogWrite(conf.traction_enb, 0);
    return ;    
  }
  if (pwm < conf.pwm_neutral_val)
  {
    // GO REVERSE
    pwm = (conf.pwm_neutral_val - pwm) * 2;
    digitalWrite(conf.traction_in3, HIGH);
    digitalWrite(conf.traction_in4, LOW);
    analogWrite(conf.traction_enb, pwm);
  }
  else
  {
    // GO FORWARD
    pwm = map(pwm, 128, 255, 0, 255);
    digitalWrite(conf.traction_in3, LOW);
    digitalWrite(conf.traction_in4, HIGH);
    analogWrite(conf.traction_enb, pwm);
  }
}

void stopMotors() {
  // Stop steering motor
  digitalWrite(conf.steering_in1, LOW);
  digitalWrite(conf.steering_in2, LOW);
  analogWrite(conf.steering_ena, 0);
  
  // Stop traction motor
  digitalWrite(conf.traction_in3, LOW);
  digitalWrite(conf.traction_in4, LOW);
  analogWrite(conf.traction_enb, 0);
}