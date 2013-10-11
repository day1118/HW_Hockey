/*
  Hockey Warrior - IR Test
  Plays hockey
 */
 
  #include "config.h"
  #include "states.h"
  #include "thresholds.h"
  #include <Servo.h> 
  #include <NewPing.h> 

NewPing US_back(BACK_ULTRASONIC_SENSOR_TRIG, BACK_ULTRASONIC_SENSOR_ECHO, MAX_ULTRASONIC_DISTANCE_CM);
NewPing US_front(FRONT_ULTRASONIC_SENSOR_TRIG, FRONT_ULTRASONIC_SENSOR_ECHO, MAX_ULTRASONIC_DISTANCE_CM);

Servo servoF;  // create servo object to control a servo 
Servo servoB;
Servo servoK;
 
int IRFL_FRONT_Off, IRFL_FRONT_On, IRFL_SIDE_Off, IRFL_SIDE_On, IRFL_FRONT_Diff, IRFL_SIDE_Diff;
int IRFR_FRONT_Off, IRFR_FRONT_On, IRFR_SIDE_Off, IRFR_SIDE_On, IRFR_FRONT_Diff, IRFR_SIDE_Diff;
int IRBL_BACK_Off, IRBL_BACK_On, IRBL_SIDE_Off, IRBL_SIDE_On, IRBL_BACK_Diff, IRBL_SIDE_Diff;
int IRBR_BACK_Off, IRBR_BACK_On, IRBR_SIDE_Off, IRBR_SIDE_On, IRBR_BACK_Diff, IRBR_SIDE_Diff;

int MICRO_FRONT_Left, MICRO_FRONT_Right, MICRO_BACK_Left, MICRO_BACK_Right;

int BALL_Off, BALL_RED_On, BALL_IR_On, BALL_RED_Diff, BALL_IR_Diff;
int GML_Off, GML_RED_On, GML_GREEN_On, GML_RED_Diff, GML_GREEN_Diff;
int GMR_Off, GMR_RED_On, GMR_GREEN_On, GMR_RED_Diff, GMR_GREEN_Diff;
  
int defaultMotorSpeed = 220;

int driveState = STATE_DRIVE_FORWARDS;
int overallState = STATE_OVERALL_SEARCH_BALL;
int motorLSpeed = defaultMotorSpeed;
int motorRSpeed = defaultMotorSpeed;

int servoFPos, servoBPos, servoKPos;
int servoState = STATE_SERVO_SERACH;

int goalState = STATE_GOAL_DRIVE_OVER_MAT;

int greenMatLeftCount = 0, greenMatRightCount = 0;

int ballType, greenMatLeftState, greenMatRightState;

int CAM_startPoint, CAM_conseq, CAM_bestStart, CAM_bestWidth, CAM_center, CAM_direction;

unsigned long driveTimer = 0, servoTimer = 0, goalTimer = 0;

unsigned int US_back_cm;
unsigned int US_front_cm;

int averageCount = 1;

void setup() {
	// Set IR pins as outputs
  pinMode(IRFL_IR_LED_PIN, OUTPUT);
  pinMode(IRFR_IR_LED_PIN, OUTPUT);
  pinMode(IRBL_IR_LED_PIN, OUTPUT);
  pinMode(IRBR_IR_LED_PIN, OUTPUT);

  pinMode(GREEN_MAT_LEFT_RED_LED_PIN, OUTPUT);
  pinMode(GREEN_MAT_LEFT_GREEN_LED_PIN, OUTPUT);
  pinMode(GREEN_MAT_RIGHT_RED_LED_PIN, OUTPUT);
  pinMode(GREEN_MAT_RIGHT_GREEN_LED_PIN, OUTPUT);
  pinMode(BALL_COLOUR_RED_LED_PIN, OUTPUT);
  pinMode(BALL_COLOUR_IR_LED_PIN, OUTPUT);

  pinMode(MOTOR_L_A_PIN, OUTPUT);
  pinMode(MOTOR_L_B_PIN, OUTPUT);
  pinMode(MOTOR_L_ENABLE_PIN, OUTPUT);

  pinMode(MOTOR_R_A_PIN, OUTPUT);
  pinMode(MOTOR_R_B_PIN, OUTPUT);
  pinMode(MOTOR_R_ENABLE_PIN, OUTPUT);

  pinMode(CAMERA_CLK_PIN, OUTPUT);
  pinMode(CAMERA_SI_PIN, OUTPUT);

  digitalWrite(CAMERA_CLK_PIN, LOW);

  digitalWrite(MOTOR_L_A_PIN, HIGH);
  digitalWrite(MOTOR_L_B_PIN, LOW);
  digitalWrite(MOTOR_R_A_PIN, HIGH);
  digitalWrite(MOTOR_R_B_PIN, LOW);

  analogWrite(MOTOR_L_ENABLE_PIN, motorLSpeed);
  analogWrite(MOTOR_R_ENABLE_PIN, motorRSpeed);

  servoF.attach(SERVO_FRONT_PIN, SERVO_FRONT_MIN_PWM, SERVO_FRONT_MAX_PWM);  // attaches the servo on pin 9 to the servo object 
  servoB.attach(SERVO_BACK_PIN, SERVO_BACK_MIN_PWM, SERVO_BACK_MAX_PWM);
  servoK.attach(SERVO_KICK_PIN, SERVO_KICK_MIN_PWM, SERVO_KICK_MAX_PWM);

  Serial.begin(115200);      // open the serial port at 9600 bps:

  DEBUG_PRINT("Starting...");
}

void loop() {
  // Read each sensor in a loop
  readIRSensors();
  readTouchSensors();
  setMotors();
  readColourSensors();
  setServos();
  readUSSensors();
  
  #ifdef PLOT_PRINT_STATUS_ON
    PLOT("overallState", overallState);
    PLOT("driveState", driveState);
    PLOT("goalState", goalState);
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
    if(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON)
    {
      overallState = STATE_OVERALL_ALIGN_GOAL;
      goalState = STATE_GOAL_DRIVE_OVER_MAT;
      goalTimer = millis() + TIMER_GOAL_DRIVE_OVER_MAT;
    }
    else
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
  }
  else if(overallState == STATE_OVERALL_ALIGN_GOAL)
  {
    readCamera();

    switch(goalState)
    {
      case STATE_GOAL_DRIVE_OVER_MAT:
        motorLeft(BACKWARDS);
        motorRight(BACKWARDS);

        if(goalTimer < millis())
        {
          if(greenMatLeftState == GREEN_MAT_ON)
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          else
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
          }
        }
        break;

        case STATE_GOAL_ROTATE_LEFT:
          motorLeft(FORWARDS);
          motorRight(BACKWARDS);

          if(CAM_direction == BEACON_CENTER || goalTimer < millis())
          {
            goalState = STATE_GOAL_BACKOFF;
            goalTimer = millis() + TIMER_GOAL_BACKOFF; 
          }
          else if(CAM_direction == BEACON_RIGHT)
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
          }
          break;

        case STATE_GOAL_ROTATE_RIGHT:
          motorLeft(BACKWARDS);
          motorRight(FORWARDS);

          if(CAM_direction == BEACON_CENTER || goalTimer < millis())
          {
            goalState = STATE_GOAL_BACKOFF;
            goalTimer = millis() + TIMER_GOAL_BACKOFF; 
          }
          else if(CAM_direction == BEACON_LEFT)
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          break;

        case STATE_GOAL_BACKOFF:
          motorLeft(FORWARDS);
          motorRight(FORWARDS);

          if(!(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON) || goalTimer < millis())
          {
            goalState = STATE_GOAL_KICK;
          }
          break;

        case STATE_GOAL_KICK:
          motorLeft(STOP);
          motorRight(STOP);
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
  digitalWrite(GREEN_MAT_LEFT_RED_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  GML_RED_On = analogRead(GREEN_MAT_LEFT_PHOTOTRANSISTOR_PIN);
  digitalWrite(GREEN_MAT_LEFT_RED_LED_PIN, LOW);

  digitalWrite(GREEN_MAT_LEFT_GREEN_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  GML_GREEN_On = analogRead(GREEN_MAT_LEFT_PHOTOTRANSISTOR_PIN);
  digitalWrite(GREEN_MAT_LEFT_GREEN_LED_PIN, LOW);

  digitalWrite(GREEN_MAT_RIGHT_RED_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  GMR_RED_On = analogRead(GREEN_MAT_RIGHT_PHOTOTRANSISTOR_PIN);
  digitalWrite(GREEN_MAT_RIGHT_RED_LED_PIN, LOW);

  digitalWrite(GREEN_MAT_RIGHT_GREEN_LED_PIN, HIGH);
  delay(LED_READ_DELAY_TIME);
  GMR_GREEN_On = analogRead(GREEN_MAT_RIGHT_PHOTOTRANSISTOR_PIN);
  digitalWrite(GREEN_MAT_RIGHT_GREEN_LED_PIN, LOW);

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

  GML_RED_Diff = GML_RED_On - GML_Off;
  GML_GREEN_Diff = GML_GREEN_On - GML_Off;
  GMR_RED_Diff = GMR_RED_On - GMR_Off;
  GMR_GREEN_Diff = GMR_GREEN_On - GMR_Off;
  BALL_RED_Diff = BALL_RED_On - BALL_Off;
  BALL_IR_Diff = BALL_IR_On - BALL_Off;

  ballType = determineBallType();

  greenMatLeftState = GREEN_MAT_OFF; // determineGMLState();
  greenMatRightState = determineGMRState();

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("GML_Off", GML_Off);
    PLOT("GML_RED_On", GML_RED_On);
    PLOT("GML_GREEN_On", GML_GREEN_On);
    PLOT("GML_RED_Diff", GML_RED_Diff);
    PLOT("GML_GREEN_Diff", GML_GREEN_Diff);

    PLOT("GMR_Off", GMR_Off);
    PLOT("GMR_RED_On", GMR_RED_On);
    PLOT("GMR_GREEN_On", GMR_GREEN_On);
    PLOT("GMR_RED_Diff", GMR_RED_Diff);
    PLOT("GMR_GREEN_Diff", GMR_GREEN_Diff);

    PLOT("BALL_Off", BALL_Off);
    PLOT("BALL_RED_On", BALL_RED_On);
    PLOT("BALL_IR_On", BALL_IR_On);
    PLOT("BALL_RED_Diff", BALL_RED_Diff);
    PLOT("BALL_IR_Diff", BALL_IR_Diff);
  #endif
}

void setServos()
{
  switch(servoState)
  {
    case STATE_SERVO_SERACH:
      servoFPos = SERVO_FRONT_UP;
      servoBPos = SERVO_BACK_DOWN;
      servoKPos = SERVO_KICK_UP;

      if(ballType == BALL_WRONG)
      {
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
        servoState = STATE_SERVO_WRONG_BALL;
      }
      else if(ballType == BALL_RIGHT)
      {  
        servoTimer = millis() + TIMER_SERVO_RIGHT_BALL;
        servoState = STATE_SERVO_RIGHT_BALL_DELAY;
      }
      break;

    case STATE_SERVO_WRONG_BALL:
      servoFPos = SERVO_FRONT_DOWN;
      servoBPos = SERVO_BACK_UP;
      servoKPos = SERVO_KICK_UP;

      if(ballType == BALL_WRONG)
      {
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
      }
      if(servoTimer < millis())
      {
        servoState = STATE_SERVO_SERACH;
      }
      break;

    case STATE_SERVO_RIGHT_BALL_DELAY:
      servoFPos = SERVO_FRONT_DOWN;
      servoBPos = SERVO_BACK_DOWN;
      servoKPos = SERVO_KICK_UP;

      if(ballType == BALL_WRONG)
      {
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
        servoState = STATE_SERVO_WRONG_BALL;
      }

      if(servoTimer < millis())
      {
        overallState = STATE_OVERALL_SEARCH_GOAL;
        driveState = STATE_DRIVE_FORWARDS;
        servoState = STATE_SERVO_RIGHT_BALL;
      }
      break;
    
    case STATE_SERVO_RIGHT_BALL:
      servoFPos = SERVO_FRONT_DOWN;
      servoBPos = SERVO_BACK_DOWN;
      servoKPos = SERVO_KICK_UP;

      if(ballType == BALL_WRONG)
      {
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
        servoState = STATE_SERVO_WRONG_BALL;
      }

      // This operation should be done by the overallState. This is just for testing.
      //if(servoTimer < millis())
      if(overallState == STATE_OVERALL_ALIGN_GOAL && goalState == STATE_GOAL_KICK)
      {
        servoTimer = millis() + TIMER_SERVO_KICK_1_DELAY;
        servoState = STATE_SERVO_KICK_1_DELAY;
      }
      break;

    case STATE_SERVO_KICK_1_DELAY:
      servoFPos = SERVO_FRONT_DOWN;
      servoBPos = SERVO_BACK_DOWN;
      servoKPos = SERVO_KICK_UP;

      if(servoTimer < millis())
      {
        servoTimer = millis() + TIMER_SERVO_KICK_1;
        servoState = STATE_SERVO_KICK_1;
      }
      break;

    case STATE_SERVO_KICK_1:
      servoFPos = SERVO_FRONT_UP;
      servoBPos = SERVO_BACK_UP;
      servoKPos = SERVO_KICK_UP;

      if(servoTimer < millis())
      {
        servoTimer = millis() + TIMER_SERVO_KICK_2;
        servoState = STATE_SERVO_KICK_2;
      }
      break;

    case STATE_SERVO_KICK_2:
      servoFPos = SERVO_FRONT_UP;
      servoBPos = SERVO_BACK_UP;
      servoKPos = SERVO_KICK_DOWN;

      if(servoTimer < millis())
      {
        servoTimer = millis() + TIMER_SERVO_KICK_3;
        servoState = STATE_SERVO_KICK_3;
      }
      break;

    case STATE_SERVO_KICK_3:
      servoFPos = SERVO_FRONT_UP;
      servoBPos = SERVO_BACK_UP;
      servoKPos = SERVO_KICK_UP;

      if(servoTimer < millis())
      {
        overallState = STATE_OVERALL_SEARCH_BALL;
        driveState = STATE_DRIVE_FORWARDS;
        servoState = STATE_SERVO_SERACH;
      }
      break;
  }

  servoF.write(servoFPos);
  servoB.write(servoBPos);
  servoK.write(servoKPos);

  #ifdef PLOT_PRINT_SERVOS_ON
    PLOT("servoFPos", servoFPos);
    PLOT("servoBPos", servoBPos);
    PLOT("servoKPos", servoKPos);

    PLOT("servoState", servoState);
    PLOT("servoTimer", servoTimer);
  #endif
}

int determineBallType()
{
  int ballColour;

  if(BALL_IR_Diff > BALL_IR_Thresh)
  {
    if(BALL_RED_Diff > BALL_RED_Thresh)
      ballColour = BALL_RED;
    else
      ballColour = BALL_BLUE;

    if(DESIRED_BALL_COLOUR == ballColour)
      ballColour = BALL_RIGHT;
    else
      ballColour = BALL_WRONG;
  }
  else
    ballColour = BALL_NONE;

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("ballColour", ballColour);
  #endif  

  return ballColour;
}

int determineGMLState()
{
  int greenMatLeftState, greenMatLeftStateRaw;

  if(GML_GREEN_Diff < GML_GREEN_HIGH_Thres && GML_GREEN_Diff > GML_GREEN_LOW_Thres && GML_RED_Diff < GML_RED_HIGH_Thres )
  {
    greenMatLeftStateRaw = GREEN_MAT_ON;
    greenMatLeftCount++;
  }
  else
  {
    greenMatLeftStateRaw = GREEN_MAT_OFF;
    greenMatLeftCount--;
  }

  if(greenMatLeftCount > GM_FILTER_SIZE)    
    greenMatLeftCount = GM_FILTER_SIZE;
  if(greenMatLeftCount < 0)    
    greenMatLeftCount = 0;

  if(greenMatLeftCount > GM_FILTER_SIZE/2)
    greenMatLeftState = GREEN_MAT_ON;
  else
    greenMatLeftState = GREEN_MAT_OFF;

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("greenMatLeftStateRaw", greenMatLeftStateRaw);
    PLOT("greenMatLeftState", greenMatLeftState);
  #endif  

  return greenMatLeftState;
}

int determineGMRState()
{
  int greenMatRightStateTemp, greenMatRightStateRaw;

  if(GMR_GREEN_Diff < GMR_GREEN_HIGH_Thres && GMR_GREEN_Diff > GMR_GREEN_LOW_Thres && GMR_RED_Diff < GML_RED_HIGH_Thres )
  {
    greenMatRightStateRaw = GREEN_MAT_ON;
    greenMatRightCount++;
  }
  else
  {
    greenMatRightStateRaw = GREEN_MAT_OFF;
    greenMatRightCount--;
  }

  if(greenMatRightCount > GM_FILTER_SIZE)    
    greenMatRightCount = GM_FILTER_SIZE;

  if(greenMatRightCount < 0)    
    greenMatRightCount = 0;

  if(greenMatRightCount > GM_FILTER_SIZE/2)
    greenMatRightStateTemp = GREEN_MAT_ON;
  else
    greenMatRightStateTemp = GREEN_MAT_OFF;

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("greenMatRightStateRaw", greenMatRightStateRaw);
    PLOT("greenMatRightStateTemp", greenMatRightStateTemp);
  #endif  

  return greenMatRightStateTemp;
}

int readCamera()
{
  int sensorValue;

  digitalWrite(CAMERA_SI_PIN, HIGH);
  delayMicroseconds(CAMERA_DELAY_TIME/2);               // wait for a second
  digitalWrite(CAMERA_CLK_PIN, HIGH);
  delayMicroseconds(CAMERA_DELAY_TIME/2);               // wait for a second
  digitalWrite(CAMERA_SI_PIN, LOW);
  delayMicroseconds(CAMERA_DELAY_TIME/2);
  
  for(int i = 0; i < 130; i ++)
  {
    digitalWrite(CAMERA_CLK_PIN, HIGH);
    delayMicroseconds(1);               // wait for a second
    digitalWrite(CAMERA_CLK_PIN, LOW);
    delayMicroseconds(1);               // wait for a second
  }
  
  delayMicroseconds(100);               // wait for a second

  digitalWrite(CAMERA_SI_PIN, HIGH);
  delayMicroseconds(CAMERA_DELAY_TIME/2);               // wait for a second
  digitalWrite(CAMERA_CLK_PIN, HIGH);
  delayMicroseconds(CAMERA_DELAY_TIME/2);               // wait for a second
  digitalWrite(CAMERA_SI_PIN, LOW);
  delayMicroseconds(CAMERA_DELAY_TIME/2);
  
  CAM_startPoint = 0;
  CAM_conseq = 0;
  CAM_bestStart = 0;
  CAM_bestWidth = 0;

  #ifdef PLOT_PRINT_CAMERA_ON
    PLOT_PRINT("CAM_RAW");
  #endif
  
  for(int i = 0; i < 130; i ++)
  {
    digitalWrite(CAMERA_CLK_PIN, HIGH);
    //delayMicroseconds(CAMERA_DELAY_TIME/2);               // wait for a second
    sensorValue = analogRead(CAMERA_ANALOG_IN_PIN);
    #ifdef PLOT_PRINT_CAMERA_ON
      PLOT_PRINT(":");
      PLOT_PRINT(sensorValue);
    #endif

    digitalWrite(CAMERA_CLK_PIN, LOW);

    if(sensorValue > 800)
    {
      if(CAM_conseq == 0)
      {
         CAM_startPoint = i;
      }
    CAM_conseq ++;
      if(CAM_conseq > CAM_bestWidth)
      {
         CAM_bestStart = CAM_startPoint;
         CAM_bestWidth = CAM_conseq;
      }
    }
    else
    {
      CAM_conseq = 0;
    }
  }
 
 #ifdef PLOT_PRINT_CAMERA_ON
    PLOT_PRINTLN("");
  #endif

  if(CAM_bestWidth > 10)  
  {
    CAM_center = CAM_bestStart + (CAM_bestWidth/2);

    if(CAM_center < ((128/2) + 40) && CAM_center > ((128/2) - 40))
      CAM_direction = BEACON_CENTER;
    else if(CAM_center < (128/2))
      CAM_direction = BEACON_LEFT;
    else if(CAM_center > (128/2))
      CAM_direction = BEACON_RIGHT;
  }
  else
  {
    CAM_center = -1;
    CAM_direction = BEACON_NONE;
  }

  #ifdef PLOT_PRINT_CAMERA_ON
    PLOT("CAM_Start", CAM_bestStart);
    PLOT("CAM_Width", CAM_bestWidth);
  #endif
}

void readUSSensors()
{
  US_back_cm = US_back.ping_cm();
  US_front_cm = US_front.ping_cm();  

  #ifdef PLOT_PRINT_US_ON
    PLOT("US_back_cm", US_back_cm);
    PLOT("US_front_cm", US_front_cm);
  #endif
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
