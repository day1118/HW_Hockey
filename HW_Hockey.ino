/*
  Hockey Warrior - IR Test
  Plays hockey
 */
 
  #include "config.h"
  #include "states.h"
  #include "thresholds.h"
  
int IRFL_FRONT_Off, IRFL_FRONT_On, IRFL_SIDE_Off, IRFL_SIDE_On, IRFL_FRONT_Diff, IRFL_SIDE_Diff;
int IRFR_FRONT_Off, IRFR_FRONT_On, IRFR_SIDE_Off, IRFR_SIDE_On, IRFR_FRONT_Diff, IRFR_SIDE_Diff;
int IRBL_BACK_Off, IRBL_BACK_On, IRBL_SIDE_Off, IRBL_SIDE_On, IRBL_BACK_Diff, IRBL_SIDE_Diff;
int IRBR_BACK_Off, IRBR_BACK_On, IRBR_SIDE_Off, IRBR_SIDE_On, IRBR_BACK_Diff, IRBR_SIDE_Diff;

int MICRO_FRONT_Left, MICRO_FRONT_Right, MICRO_BACK_Left, MICRO_BACK_Right;

int BALL_Off, BALL_RED_On, BALL_IR_On, BALL_RED_Diff, BALL_IR_Diff;
  
int defaultMotorSpeed = 220;

int driveState = STATE_DRIVE_FORWARDS;
int overallState = STATE_OVERALL_SEARCH_GOAL;
int motorLSpeed = defaultMotorSpeed;
int motorRSpeed = defaultMotorSpeed;

unsigned long driveTimer = 0;

int averageCount = 1;

void setup() {
	// Set IR pins as outputs
  pinMode(IRFL_IR_LED_PIN, OUTPUT);
  pinMode(IRFR_IR_LED_PIN, OUTPUT);
  pinMode(IRBL_IR_LED_PIN, OUTPUT);
  pinMode(IRBR_IR_LED_PIN, OUTPUT);

  pinMode(BALL_COLOUR_RED_LED_PIN, OUTPUT);
  pinMode(BALL_COLOUR_IR_LED_PIN, OUTPUT);

  pinMode(MOTOR_L_A_PIN, OUTPUT);
  pinMode(MOTOR_L_B_PIN, OUTPUT);
  pinMode(MOTOR_L_ENABLE_PIN, OUTPUT);

  pinMode(MOTOR_R_A_PIN, OUTPUT);
  pinMode(MOTOR_R_B_PIN, OUTPUT);
  pinMode(MOTOR_R_ENABLE_PIN, OUTPUT);

  digitalWrite(MOTOR_L_A_PIN, HIGH);
  digitalWrite(MOTOR_L_B_PIN, LOW);
  digitalWrite(MOTOR_R_A_PIN, HIGH);
  digitalWrite(MOTOR_R_B_PIN, LOW);

  analogWrite(MOTOR_L_ENABLE_PIN, motorLSpeed);
  analogWrite(MOTOR_R_ENABLE_PIN, motorRSpeed);

  Serial.begin(115200);      // open the serial port at 9600 bps:

  DEBUG_PRINT("Starting...");
}

void loop() {
  // Read each sensor in a loop
  readIRSensors();
  readTouchSensors();
  setMotors();
  readColourSensors();
  
  #ifdef PLOT_PRINT_STATUS_ON
    PLOT("overallState", overallState);
    PLOT("millis", millis());
  #endif
  //PLOT_OVERRIDE("freeMemory", freeRam());
}


int readSensor(int pin, int averageCount)
{
  int value = 0;
  int i = 0;
  for(i = 0; i < averageCount; i++)
      value += analogRead(pin);
  return value/averageCount;
}

int readDigitalSensor(int pin, int averageCount)
{
  int value = 0;
  int i = 0;
  for(i = 0; i < averageCount; i++)
      value += digitalRead(pin);
  return value/averageCount;
}

void readIRSensors()
{
  digitalWrite(IRFL_IR_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  IRFL_FRONT_On = readSensor(IRFL_FRONT_PHOTOTRANSISTOR_PIN, averageCount);
  IRFL_SIDE_On = readSensor(IRFL_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  digitalWrite(IRFL_IR_LED_PIN, LOW);

  digitalWrite(IRFR_IR_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  IRFR_FRONT_On = readSensor(IRFR_FRONT_PHOTOTRANSISTOR_PIN, averageCount);
  IRFR_SIDE_On = readSensor(IRFR_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  digitalWrite(IRFR_IR_LED_PIN, LOW);

  digitalWrite(IRBL_IR_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  IRBL_BACK_On = readSensor(IRBL_BACK_PHOTOTRANSISTOR_PIN, averageCount);
  IRBL_SIDE_On = readSensor(IRBL_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  digitalWrite(IRBL_IR_LED_PIN, LOW);

  digitalWrite(IRBR_IR_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  IRBR_BACK_On = readSensor(IRBR_BACK_PHOTOTRANSISTOR_PIN, averageCount);
  IRBR_SIDE_On = readSensor(IRBR_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  digitalWrite(IRBR_IR_LED_PIN, LOW);  
    
  IRFL_FRONT_Off = readSensor(IRFL_FRONT_PHOTOTRANSISTOR_PIN, averageCount);
  IRFL_SIDE_Off = readSensor(IRFL_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  IRFR_FRONT_Off = readSensor(IRFR_FRONT_PHOTOTRANSISTOR_PIN, averageCount);
  IRFR_SIDE_Off = readSensor(IRFR_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  
  IRBL_BACK_Off = readSensor(IRBL_BACK_PHOTOTRANSISTOR_PIN, averageCount);
  IRBL_SIDE_Off = readSensor(IRBL_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  IRBR_BACK_Off = readSensor(IRBR_BACK_PHOTOTRANSISTOR_PIN, averageCount);
  IRBR_SIDE_Off = readSensor(IRBR_SIDE_PHOTOTRANSISTOR_PIN, averageCount);
  
  IRFL_FRONT_Diff = IRFL_FRONT_On - IRFL_FRONT_Off;
  IRFL_SIDE_Diff = IRFL_SIDE_On - IRFL_SIDE_Off;
  IRFR_FRONT_Diff = IRFR_FRONT_On - IRFR_FRONT_Off;
  IRFR_SIDE_Diff = IRFR_SIDE_On - IRFR_SIDE_Off;
  IRBL_BACK_Diff = IRBL_BACK_On - IRBL_BACK_Off;
  IRBL_SIDE_Diff = IRBL_SIDE_On - IRBL_SIDE_Off;
  IRBR_BACK_Diff = IRBR_BACK_On - IRBR_BACK_Off;
  IRBR_SIDE_Diff = IRBR_SIDE_On - IRBR_SIDE_Off;
  
  #ifdef PLOT_PRINT_IR_ON
    PLOT("IRFL_FRONT_Off", IRFL_FRONT_Off);
    PLOT("IRFL_FRONT_On", IRFL_FRONT_On);
    PLOT("IRFL_SIDE_Off", IRFL_SIDE_Off);
    PLOT("IRFL_SIDE_On", IRFL_SIDE_On);
    PLOT("IRFL_FRONT_Diff", IRFL_FRONT_Diff);
    PLOT("IRFL_SIDE_Diff", IRFL_SIDE_Diff);
    
    PLOT("IRFR_FRONT_Off", IRFR_FRONT_Off);
    PLOT("IRFR_FRONT_On", IRFR_FRONT_On);
    PLOT("IRFR_SIDE_Off", IRFR_SIDE_Off);
    PLOT("IRFR_SIDE_On", IRFR_SIDE_On);
    PLOT("IRFR_FRONT_Diff", IRFR_FRONT_Diff);
    PLOT("IRFR_SIDE_Diff", IRFR_SIDE_Diff);
    
    PLOT("IRBL_BACK_Off", IRBL_BACK_Off);
    PLOT("IRBL_BACK_On", IRBL_BACK_On);
    PLOT("IRBL_SIDE_Off", IRBL_SIDE_Off);
    PLOT("IRBL_SIDE_On", IRBL_SIDE_On);
    PLOT("IRBL_BACK_Diff", IRBL_BACK_Diff);
    PLOT("IRBL_SIDE_Diff", IRBL_SIDE_Diff);
    
    PLOT("IRBR_BACK_Off", IRBR_BACK_Off);
    PLOT("IRBR_BACK_On", IRBR_BACK_On);
    PLOT("IRBR_SIDE_Off", IRBR_SIDE_Off);
    PLOT("IRBR_SIDE_On", IRBR_SIDE_On);
    PLOT("IRBR_BACK_Diff", IRBR_BACK_Diff);
    PLOT("IRBR_SIDE_Diff", IRBR_SIDE_Diff);
  #endif
}

void readTouchSensors()
{
  MICRO_FRONT_Left = readDigitalSensor(FRONT_LEFT_TOUCH_SENSOR, averageCount);
  MICRO_FRONT_Right = readDigitalSensor(FRONT_RIGHT_TOUCH_SENSOR, averageCount);

  MICRO_BACK_Left = readDigitalSensor(BACK_LEFT_TOUCH_SENSOR, averageCount);
  MICRO_BACK_Right = readDigitalSensor(BACK_RIGHT_TOUCH_SENSOR, averageCount);
  
  #ifdef PLOT_PRINT_TOUCH_ON
    PLOT("MICRO_FRONT_Left", MICRO_FRONT_Left);
    PLOT("MICRO_FRONT_Right", MICRO_FRONT_Right);
  
    PLOT("MICRO_BACK_Left", MICRO_BACK_Left);
    PLOT("MICRO_BACK_Right", MICRO_BACK_Right);
  #endif
}

void setMotors()
{
  if(overallState == STATE_OVERALL_SEARCH_BALL)
  {
    switch(driveState)
    {
      case STATE_DRIVE_FORWARDS:
        motorLeft(FORWARDS);
        motorRight(FORWARDS);

        if(!MICRO_FRONT_Left)
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
        }
        if(!MICRO_FRONT_Right)
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
        }

        if(IRFR_FRONT_Diff > IRFL_FRONT_Thresh)
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
        }
        else if(IRFL_FRONT_Diff > IRFR_FRONT_Thresh)
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
        }
        else if(IRFL_SIDE_Diff > IRFL_SIDE_Thresh)
        {
          driveState = STATE_DRIVE_BEND_RIGHT;
          driveTimer = millis() + TIMER_DRIVE_BEND_RIGHT; 
        }
        else if(IRFR_SIDE_Diff > IRFR_SIDE_Thresh)
        {
          driveState = STATE_DRIVE_BEND_LEFT;
          driveTimer = millis() + TIMER_DRIVE_BEND_LEFT; 
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_BACK:
        motorLeft(BACKWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis() || !(IRFL_FRONT_Diff > IRFL_FRONT_Thresh || IRFR_FRONT_Diff > IRFR_FRONT_Thresh))
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_LEFT;
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_LEFT:
        motorLeft(BACKWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_BACK:
        motorLeft(BACKWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis() || !(IRFL_FRONT_Diff > IRFL_FRONT_Thresh || IRFR_FRONT_Diff > IRFR_FRONT_Thresh))
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
        motorLeft(FORWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_STOP:
        motorLeft(STOP);
        motorRight(STOP);

        if(IRFL_FRONT_Diff > IRFL_FRONT_Thresh || IRFR_FRONT_Diff > IRFR_FRONT_Thresh)
        {
          driveTimer = millis() + TIMER_DRIVE_STOP;
        }

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BEND_LEFT:
        motorLeft(BACKWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis() || !(IRFR_SIDE_Diff > IRFR_SIDE_Thresh))
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BEND_RIGHT:
        motorLeft(FORWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis() || !(IRFL_SIDE_Diff > IRFL_SIDE_Thresh))
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;
    }
  }
  else if(overallState == STATE_OVERALL_SEARCH_GOAL)
  {
    switch(driveState)
    {
      case STATE_DRIVE_FORWARDS:
        motorLeft(BACKWARDS);
        motorRight(BACKWARDS);

        if(!MICRO_BACK_Left)
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
        }
        else if(!MICRO_BACK_Right)
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
        }
        else if(IRBR_BACK_Diff > IRBL_BACK_Thresh)
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
        }
        else if(IRBL_BACK_Diff > IRBR_BACK_Thresh)
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
        }
        else if(IRBL_SIDE_Diff > IRBL_SIDE_Thresh)
        {
          driveState = STATE_DRIVE_BEND_RIGHT;
          driveTimer = millis() + TIMER_DRIVE_BEND_RIGHT; 
        }
        else if(IRBR_SIDE_Diff > IRBR_SIDE_Thresh)
        {
          driveState = STATE_DRIVE_BEND_LEFT;
          driveTimer = millis() + TIMER_DRIVE_BEND_LEFT; 
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_BACK:
        motorLeft(FORWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis() || !(IRBR_BACK_Diff > IRBR_BACK_Thresh || IRBR_BACK_Diff > IRBR_BACK_Thresh))
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_LEFT;
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_LEFT:
        motorLeft(FORWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_BACK:
        motorLeft(FORWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis() || !(IRBR_BACK_Diff > IRBR_BACK_Thresh || IRBR_BACK_Diff > IRBR_BACK_Thresh))
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
        motorLeft(BACKWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_STOP:
        motorLeft(STOP);
        motorRight(STOP);

        if(IRBR_BACK_Diff > IRBR_BACK_Thresh || IRBR_BACK_Diff > IRBR_BACK_Thresh)
        {
          driveTimer = millis() + TIMER_DRIVE_STOP;
        }

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BEND_LEFT:
        motorLeft(FORWARDS);
        motorRight(BACKWARDS);

        if(driveTimer < millis() || !(IRBR_SIDE_Diff > IRBR_SIDE_Thresh))
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BEND_RIGHT:
        motorLeft(BACKWARDS);
        motorRight(FORWARDS);

        if(driveTimer < millis() || !(IRBL_SIDE_Diff > IRBL_SIDE_Thresh))
        {
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;
    }
  }

  #ifdef MOTORS_ON
    analogWrite(MOTOR_L_ENABLE_PIN, motorLSpeed);
    analogWrite(MOTOR_R_ENABLE_PIN, motorRSpeed);
  #else
    analogWrite(MOTOR_L_ENABLE_PIN, 0);
    analogWrite(MOTOR_R_ENABLE_PIN, 0);
  #endif

  #ifdef PLOT_PRINT_MOTORS_ON  
    PLOT("driveState", driveState);
    PLOT("driveTimer", driveTimer);
    PLOT("motorLSpeed", motorLSpeed);
    PLOT("motorRSpeed", motorRSpeed);
  #endif
}

void motorLeft(int direction)
{
  if(direction == FORWARDS)
  {
    digitalWrite(MOTOR_L_A_PIN, HIGH);
    digitalWrite(MOTOR_L_B_PIN, LOW);
  }
  else if(direction == BACKWARDS)
  {
    digitalWrite(MOTOR_L_A_PIN, LOW);
    digitalWrite(MOTOR_L_B_PIN, HIGH); 
  }
  else
  {
    digitalWrite(MOTOR_L_A_PIN, LOW);
    digitalWrite(MOTOR_L_B_PIN, LOW); 
  }

  #ifdef PLOT_PRINT_MOTORS_ON
    PLOT("motorLeft", direction);
  #endif
}
 
void motorRight(int direction)
{
  if(direction == FORWARDS)
  {
    digitalWrite(MOTOR_R_A_PIN, LOW);
    digitalWrite(MOTOR_R_B_PIN, HIGH);
  }
  else if(direction == BACKWARDS)
  {
    digitalWrite(MOTOR_R_A_PIN, HIGH);
    digitalWrite(MOTOR_R_B_PIN, LOW);
  } 
  else
  {
    digitalWrite(MOTOR_R_A_PIN, LOW);
    digitalWrite(MOTOR_R_B_PIN, LOW);
  }

  #ifdef PLOT_PRINT_MOTORS_ON
    PLOT("motorRight", direction);
  #endif
}

void readColourSensors()
{
  digitalWrite(BALL_COLOUR_RED_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  BALL_RED_On = analogRead(BALL_COLOUR_PHOTOTRANSISTOR_PIN);
  digitalWrite(BALL_COLOUR_RED_LED_PIN, LOW);

  digitalWrite(BALL_COLOUR_IR_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  BALL_IR_On = analogRead(BALL_COLOUR_PHOTOTRANSISTOR_PIN);
  digitalWrite(BALL_COLOUR_IR_LED_PIN, LOW);

  delay(LED_READ_DELAY_TIME);
  BALL_Off = analogRead(BALL_COLOUR_PHOTOTRANSISTOR_PIN);

  BALL_RED_Diff = BALL_RED_On - BALL_Off;
  BALL_IR_Diff = BALL_IR_On - BALL_Off;
  
  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("BALL_Off", BALL_Off);
    PLOT("BALL_RED_On", BALL_RED_On);
    PLOT("BALL_IR_On", BALL_IR_On);
    PLOT("BALL_RED_Diff", BALL_RED_Diff);
    PLOT("BALL_IR_Diff", BALL_IR_Diff);
  #endif
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
