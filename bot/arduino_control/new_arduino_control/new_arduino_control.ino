typedef enum  s_lights_state
{
  OFF,
  BLINKING,
  ON
} t_lights_state;

typedef enum s_steer_direction
{
  LEFT,
  RIGHT,
  STEER_NONE
} t_steer_direction;

typedef enum s_traction_direction
{
  FORWARD,
  REVERSE,
  TRACTION_NONE
} t_traction_direction;

typedef struct  s_lighting
{
  int             brightness, fade_amount;
  t_lights_state  lights_state;
} t_lighting;

typedef struct  s_steering
{
  int               pwm, pin;
  t_steer_direction steer_direction;
  
} t_steering;

typedef struct s_traction
{
  int                   pwm, pin;
  t_traction_direction  traction_direction;
} t_traction;

const int       PWM_NEUTRAL = 127;
const int       LIGHTING_PIN = 11;

// STEERING
const int       STEERING_ENA_PIN = 9;
const int       STEERING_IN1_PIN = 8;
const int       STEERING_IN2_PIN = 7;

// TRACTION
const int       TRACTION_ENB_PIN = 3;
const int       TRACTION_IN3_PIN = 5;
const int       TRACTION_IN4_PIN = 4;


t_lighting      lighting;
t_steering      steering;
t_traction      traction;
char            buffer[32];
int             buffer_index = 0, buffer_size = 32;

void  _log_success(const char *msg)
{
  if (Serial.availableForWrite())
  {
    Serial.print("OK: ");
    Serial.print(msg);
    Serial.print("\n");
  }
}

void  _log_error(const char *msg)
{
  if (Serial.availableForWrite())
  {
    Serial.print("ERROR: ");
    Serial.print(msg);
    Serial.print("\n");
  }
}

void setup()
{
  // Enable serial
  Serial.begin(115200);
  Serial.println("Setup initiated...");

  // Initialize the robot
  __setup();

  _log_success("Setup complete");
}

void loop()
{
  if (Serial.available()) {
    char c = Serial.read();
    
    // Add character to buffer if not end of command
    if (c != '\n' && buffer_index < buffer_size - 1) {
      buffer[buffer_index++] = c;
    } else {
      // Null terminate the string
      buffer[buffer_index] = '\0';
      
      // Process the command
      processCommand(buffer);
      
      // Reset buffer
      buffer_index = 0;
    }
  }

  // Show adequate lights
  display_lights();
}

void  processCommand(const char* cmd) {
  if (!strncmp(cmd, "INIT", 4))
    __init_robot();
  else if (!strncmp(cmd, "DISCONNECT", 10))
    __disconnect_robot();
  else if (!strncmp(cmd, "STOP", 4))
  {
    mob_stop_motors();
    _log_success("Robot stopped.");
  }
  else if (cmd[0] == 'M')
    _parse_motor_command(cmd);
  else if (cmd[0] == 'L')
    _parse_lighting_command(cmd);
  else
  {
    mob_stop_motors();
    _log_error("Unrecognized command.");
  }
}

bool is_valid_lighting_command(const char *ptr)
{
  return (!strncmp(ptr, "OFF", 3) || !strncmp(ptr, "BLINK", 5) || !strncmp(ptr, "ON", 2) || !strncmp(ptr, "TOGGLE", 6)); 
}

void  _parse_lighting_command(const char *cmd)
{  
  char* ptr = strchr(cmd, ',');
  if (ptr)
  {
    ptr++;
    if (!is_valid_lighting_command(ptr))
    {
      _log_error("Unrecognized lighting command");
      return ;
    }
    if (!strncmp(ptr, "OFF", 3))
      __lig_lights_off();
    else if (!strncmp(ptr, "BLINK", 5))
      __lig_lights_init();
    else if (!strncmp(ptr, "ON", 2))
      __lig_lights_on();
    else if (!strncmp(ptr, "TOGGLE", 6))
      __lig_lights_toggle();
    _log_success("Lighting command executed.");
  }
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
  mob_turn(steering_pwm);
  mob_go(traction_pwm);
  _log_success("Motor command executed.");
}

void  display_lights(void)
{
  switch (lighting.lights_state)
  {
    case BLINKING:
      __lig_display_init_lights();
      break ;
    case ON:
      __lig_display_on_lights();
      break ;
    case OFF:
      __lig_display_no_lights();
      break ;
    default:
      break ;
  }
}

void  __setup(void)
{
  lighting.brightness = 0;
  lighting.fade_amount = 5;
  lighting.lights_state = OFF;

  pinMode(LIGHTING_PIN, OUTPUT);
  pinMode(STEERING_ENA_PIN, OUTPUT);
  pinMode(STEERING_IN1_PIN, OUTPUT);
  pinMode(STEERING_IN2_PIN, OUTPUT);
}

void  __init_robot(void)
{
  mob_stop_motors();
  __lig_display_no_lights();

  delay(300);
  __lig_lights_init();
  _log_success("The robot has been initialized");
}

void  __disconnect_robot(void)
{
  __lig_lights_off();
  mob_stop_motors();
  _log_success("he robot has disconnected");
}

void  __lig_lights_toggle(void)
{
  switch (lighting.lights_state)
  {
    case OFF:
      __lig_lights_init();
      break ;
    case BLINKING:
      __lig_lights_on();
      break ;
    case ON:
      __lig_lights_off();
      break ;
  }
}

// LIGHTING
void  __lig_lights_off(void)
{
  // TODO: Fix the bug with lighting not starting at 0 brightness
  lighting.lights_state = OFF;
  Serial.println("Lights turned off");
}

void  __lig_lights_init(void)
{
  lighting.lights_state = BLINKING;
  Serial.println("Lights blinking.");
}

void  __lig_lights_on(void)
{
  lighting.lights_state = ON;
  Serial.println("Lights turned on.");
}

void  __lig_display_init_lights(void)
{
  analogWrite(LIGHTING_PIN, lighting.brightness);

  // change the brightness for next time through the loop:
  lighting.brightness = lighting.brightness + lighting.fade_amount;

  // reverse the direction of the fading at the ends of the fade:
  if (lighting.brightness <= 0 || lighting.brightness >= 255) {
    lighting.fade_amount = -lighting.fade_amount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}

void  __lig_display_on_lights(void)
{
  digitalWrite(LIGHTING_PIN, HIGH);
}

void  __lig_display_no_lights(void)
{
  digitalWrite(LIGHTING_PIN, LOW);
}

// MOBILITY
void  mob_turn_left(int pwm)
{
  digitalWrite(STEERING_IN1_PIN, LOW);
  digitalWrite(STEERING_IN2_PIN, HIGH);
  digitalWrite(STEERING_ENA_PIN, pwm);
}

void  mob_turn_right(int pwm)
{
  digitalWrite(STEERING_IN1_PIN, HIGH);
  digitalWrite(STEERING_IN2_PIN, LOW);
  digitalWrite(STEERING_ENA_PIN, pwm);
}

void  mob_steer_neutral(void)
{
  digitalWrite(STEERING_IN1_PIN, LOW);
  digitalWrite(STEERING_IN2_PIN, LOW);
  digitalWrite(STEERING_ENA_PIN, 0);
}

void  mob_turn(int pwm)
{
  if (pwm == PWM_NEUTRAL)
  {
    mob_steer_neutral();
    return ;
  }
  else if (pwm < PWM_NEUTRAL)
  {
    pwm = pwm * 2;
    mob_turn_left(pwm);
  }
  else
  {
    pwm = (pwm - PWM_NEUTRAL - 1) * 2;
    mob_turn_right(pwm);
  }
}

void mob_go(int pwm)
{
  if (pwm == PWM_NEUTRAL)
  {
    mob_trac_neutral();
    return ;
  }
  else if (pwm < PWM_NEUTRAL)
  {
    pwm = (PWM_NEUTRAL - pwm) * 2;
    mob_forward(pwm);
  }
  else
  {
    pwm = (pwm - PWM_NEUTRAL - 1) * 2;
    mob_reverse(pwm);
  }
}

void  mob_forward(int pwm)
{
  digitalWrite(TRACTION_IN3_PIN, HIGH);
  digitalWrite(TRACTION_IN4_PIN, LOW);
  digitalWrite(TRACTION_ENB_PIN, pwm);
}

void  mob_reverse(int pwm)
{
  digitalWrite(TRACTION_IN3_PIN, LOW);
  digitalWrite(TRACTION_IN4_PIN, HIGH);
  digitalWrite(TRACTION_ENB_PIN, pwm);
}

void  mob_trac_neutral(void)
{
  digitalWrite(TRACTION_IN3_PIN, LOW);
  digitalWrite(TRACTION_IN4_PIN, LOW);
  digitalWrite(TRACTION_ENB_PIN, 0);
}

void  mob_stop_motors(void)
{
  // Disable traction
  mob_trac_neutral();

  // Disable steering
  mob_steer_neutral();
  Serial.println("Motors stopped.");
}