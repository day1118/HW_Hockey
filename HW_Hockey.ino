/*
  Hockey Warrior - IR Test
  Plays hockey
 */
 
  #include "Config.h"
  #include "Motors.h"
  #include "IRSensor.h"
  #include "ColourSensor.h"
  #include "TouchSensor.h"
  #include "Camera.h"
  #include "Filter.h"

  #include <Servo.h> 
  #include <NewPing.h> 

NewPing US_back(BACK_ULTRASONIC_SENSOR_TRIG, BACK_ULTRASONIC_SENSOR_ECHO, MAX_ULTRASONIC_DISTANCE_CM);
NewPing US_front(FRONT_ULTRASONIC_SENSOR_TRIG, FRONT_ULTRASONIC_SENSOR_ECHO, MAX_ULTRASONIC_DISTANCE_CM);

Servo servoF, servoB, servoK;

Motor motorLeft("MOTOR_LEFT", MOTOR_L_A_PIN, MOTOR_L_B_PIN, MOTOR_L_ENABLE_PIN);
Motor motorRight("MOTOR_RIGHT", MOTOR_R_A_PIN, MOTOR_R_B_PIN, MOTOR_R_ENABLE_PIN);
#ifdef BRUSHES_ON
  Motor motorBrushes("MOTOR_BRUSHES", MOTOR_B_A_PIN, MOTOR_B_B_PIN, MOTOR_B_ENABLE_PIN);
#else
  Motor motorBrushes("MOTOR_BRUSHES");
#endif

IRSensor IRFL("IRFL", IRFL_IR_LED_PIN, IRFL_FRONT_PHOTOTRANSISTOR_PIN, IRFL_SIDE_PHOTOTRANSISTOR_PIN, IRFL_FRONT_Thresh, IRFL_SIDE_Thresh);
IRSensor IRFR("IRFR", IRFR_IR_LED_PIN, IRFR_FRONT_PHOTOTRANSISTOR_PIN, IRFR_SIDE_PHOTOTRANSISTOR_PIN, IRFR_FRONT_Thresh, IRFR_SIDE_Thresh);
IRSensor IRBL("IRBL", IRBL_IR_LED_PIN, IRBL_BACK_PHOTOTRANSISTOR_PIN, IRBL_SIDE_PHOTOTRANSISTOR_PIN, IRBL_BACK_Thresh, IRBL_SIDE_Thresh);
IRSensor IRBR("IRBR", IRBR_IR_LED_PIN, IRBR_BACK_PHOTOTRANSISTOR_PIN, IRBR_SIDE_PHOTOTRANSISTOR_PIN, IRBR_BACK_Thresh, IRBR_SIDE_Thresh);

ColourSensor GML("GML", GREEN_MAT_LEFT_RED_LED_PIN, GREEN_MAT_LEFT_GREEN_LED_PIN, GREEN_MAT_LEFT_PHOTOTRANSISTOR_PIN);
ColourSensor GMR("GMR", GREEN_MAT_RIGHT_RED_LED_PIN, GREEN_MAT_RIGHT_GREEN_LED_PIN, GREEN_MAT_RIGHT_PHOTOTRANSISTOR_PIN);
ColourSensor BALL("BALL", BALL_COLOUR_RED_LED_PIN, BALL_COLOUR_IR_LED_PIN, BALL_COLOUR_PHOTOTRANSISTOR_PIN);

TouchSensor TFL("TFL", FRONT_LEFT_TOUCH_SENSOR);
TouchSensor TFR("TFR", FRONT_RIGHT_TOUCH_SENSOR);
TouchSensor TBL("TBL", BACK_LEFT_TOUCH_SENSOR);
TouchSensor TBR("TBR", BACK_RIGHT_TOUCH_SENSOR);

Camera camera("CAM", CAMERA_SI_PIN, CAMERA_CLK_PIN, CAMERA_ANALOG_IN_PIN);

int overallState = STATE_OVERALL_SEARCH_GOAL;
int driveState = STATE_DRIVE_FORWARDS;
int servoState = STATE_SERVO_SERACH;
int goalState = STATE_GOAL_DRIVE_OVER_MAT;

int servoFPos, servoBPos, servoKPos;
int ballType = BALL_WRONG;
int greenMatLeftState = GREEN_MAT_OFF;
int greenMatRightState = GREEN_MAT_OFF;
int CAM_direction;

bool wallFollowLeft = false;

unsigned long driveTimer = 0, servoTimer = 0, goalTimer = 0;

unsigned int US_back_cm;
unsigned int US_front_cm;

void setup() {

  pinMode(CAMERA_CLK_PIN, OUTPUT);
  pinMode(CAMERA_SI_PIN, OUTPUT);

  digitalWrite(CAMERA_CLK_PIN, LOW);

  servoF.attach(SERVO_FRONT_PIN, SERVO_FRONT_MIN_PWM, SERVO_FRONT_MAX_PWM);  // attaches the servo on pin 9 to the servo object 
  servoB.attach(SERVO_BACK_PIN, SERVO_BACK_MIN_PWM, SERVO_BACK_MAX_PWM);
  servoK.attach(SERVO_KICK_PIN, SERVO_KICK_MIN_PWM, SERVO_KICK_MAX_PWM);

  motorBrushes.stop();

  Serial.begin(115200);      // open the serial port at 9600 bps:

  DEBUG_PRINT("Starting...");
}

void loop() {
  // Read each sensor in a loop
  readIRSensors();
  readTouchSensors();
  //readColourSensors();
  readUSSensors();
  setMotors();
  setServos();
  //camera.read();
 
  #ifdef PLOT_PRINT_STATUS_ON
    PLOT("overallState", overallState);
    PLOT("driveState", driveState);
    PLOT("goalState", goalState);
    PLOT("servoState", servoState);
    PLOT("millis", millis());
  #endif
}

void readIRSensors()
{
  IRFL.update();
  IRFR.update();
  IRBL.update();
  IRBR.update();
}

void readColourSensors()
{
  GML.update();
  GMR.update();
  BALL.update();

  ballType = determineBallType();
  greenMatLeftState = determineGMLState();
  greenMatRightState = determineGMRState();
}

void readTouchSensors()
{
  TFL.update();
  TFR.update();
  TBL.update();
  TBR.update();
}

void setMotors()
{
  if(overallState == STATE_OVERALL_SEARCH_BALL)
  {
    motorBrushes.driveForwards(MOTOR_BRUSHES_NORMAL_SPEED);

    if(greenMatRightState == GREEN_MAT_ON)
    {
      //TODO - Handle avoiding goal.
      overallState = STATE_OVERALL_AVOID_GOAL;
      driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
      driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
    }
    else if(greenMatLeftState == GREEN_MAT_ON)
    {
      overallState = STATE_OVERALL_AVOID_GOAL;
      driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
      driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
    }
    else
    {
      switch(driveState)
      {
        case STATE_DRIVE_FORWARDS:
          motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(TFR.on())
          {
            driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
          }
          else if(TFL.on())
          {
            driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
          }
          else if(IRFR.getFront() > IRFR_FRONT_Thresh)
          {
            driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
          }
          else if(IRFL.getFront() > IRFL_FRONT_Thresh)
          {
            driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
          }
          else if(IRFL.getSide() > IRFL_SIDE_Thresh)
          {
            driveState = STATE_DRIVE_BEND_RIGHT;
            driveTimer = millis() + TIMER_DRIVE_BEND_RIGHT; 
          }
          else if(IRFR.getSide() > IRFR_SIDE_Thresh)
          {
            driveState = STATE_DRIVE_BEND_LEFT_LEFT;
            driveTimer = millis() + TIMER_DRIVE_BEND_LEFT_LEFT; 
          }
          break;

        case STATE_DRIVE_BACKOFF_LEFT_BACK:
          motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(driveTimer < millis() || !(IRFL.getFront() > IRFL_FRONT_Thresh || IRFR.getFront() > IRFR_FRONT_Thresh))
          {
            driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_LEFT;
          }
          break;

        case STATE_DRIVE_BACKOFF_LEFT_LEFT:
          motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(driveTimer < millis())
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          break;

        case STATE_DRIVE_BACKOFF_RIGHT_BACK:
          motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(driveTimer < millis() || !(IRFL.getFront() > IRFL_FRONT_Thresh || IRFR.getFront() > IRFR_FRONT_Thresh))
          {
            driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
            driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
          }
          break;

        case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
          motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(driveTimer < millis())
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          break;

        case STATE_DRIVE_STOP:
          motorLeft.stop();
          motorRight.stop();

          if(driveTimer < millis())
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          break;

        case STATE_DRIVE_BEND_LEFT_LEFT:
          motorLeft.stop();
          motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(!(IRFR.getSide() > IRFR_SIDE_Thresh))
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          else if(driveTimer < millis())
          {
            driveState = STATE_DRIVE_BEND_LEFT_STRAIGHT;
            driveTimer = millis() + TIMER_DRIVE_BEND_LEFT_STRAIGHT;
          }
          break;

        case STATE_DRIVE_BEND_LEFT_STRAIGHT:
          motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(!(IRFR.getSide() > IRFR_SIDE_Thresh))
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          else if(driveTimer < millis())
          {
            driveState = STATE_DRIVE_BEND_LEFT_LEFT;
            driveTimer = millis() + TIMER_DRIVE_BEND_LEFT_LEFT;
          }
          break;

        case STATE_DRIVE_BEND_RIGHT:    // No two-part bending like going left.
          motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

          if(driveTimer < millis() || !(IRFL.getSide() > IRFL_SIDE_Thresh))
          {
            driveState = STATE_DRIVE_FORWARDS;
          }
          break;
      }
    }
  }
  else if(overallState == STATE_OVERALL_SEARCH_GOAL)
  {
    motorBrushes.driveBackwards(MOTOR_BRUSHES_NORMAL_SPEED);

    if(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON)
    {
      overallState = STATE_OVERALL_ALIGN_GOAL;
      goalState = STATE_GOAL_DRIVE_OVER_MAT;
      goalTimer = millis() + TIMER_GOAL_DRIVE_OVER_MAT;
    }
    else
    {
      if(wallFollowLeft)
      {
        switch(driveState)
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(TBL.on() || TBR.on() || IRBR.getFront() > IRBR_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
            }
            else if(IRBR.getFront() > IRBR_FRONT_FAR_Thresh && !(IRBR.getSide() > IRBR_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_LEFT;
            }
            else
             if(IRBR.getSide() > IRBR_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState = STATE_DRIVE_BEND_LEFT_LEFT;
              driveTimer = millis() + TIMER_DRIVE_BEND_LEFT_LEFT;
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_BACK:
            motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis() || !(IRBL.getFront() > IRBL_FRONT_FAR_Thresh || IRBR.getFront() > IRBR_FRONT_FAR_Thresh || IRBR.getSide() > IRBR_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_LEFT;
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_LEFT:
            motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;

          case STATE_DRIVE_BEND_LEFT_LEFT:
            motorLeft.stop();
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(IRBR.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)
            { // Stalled for a long time, try again.
              // Try doing wabble again.
            }
            else if(IRBR.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState = STATE_DRIVE_BACKOFF_LEFT_BACK;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;
            }

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_BEND_LEFT_STRAIGHT;
              driveTimer = millis() + TIMER_DRIVE_BEND_LEFT_STRAIGHT;
            }
            break;

          case STATE_DRIVE_BEND_LEFT_STRAIGHT:
            motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;
        }
      }
      else
      {
        switch(driveState)
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(TBL.on() || TBR.on() || IRBL.getFront() > IRBL_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
            }
            else if(IRBL.getFront() > IRBL_FRONT_FAR_Thresh && !(IRBL.getSide() > IRBL_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
            }
            else
             if(IRBL.getSide() > IRBL_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState = STATE_DRIVE_BEND_RIGHT_RIGHT;
              driveTimer = millis() + TIMER_DRIVE_BEND_RIGHT_RIGHT;
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_BACK:
            motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis() || !(IRBL.getFront() > IRBL_FRONT_FAR_Thresh || IRBR.getFront() > IRBR_FRONT_FAR_Thresh  || IRBL.getSide() > IRBL_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
            motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_RIGHT:
            motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
            motorRight.stop();

            if(IRBL.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)
            { // Stalled for a long time, try again.
              // Try doing wabble again.
            }
            else if(IRBL.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState = STATE_DRIVE_BACKOFF_RIGHT_BACK;
              driveTimer = millis() + TIMER_DRIVE_BACKOFF_RIGHT_BACK;
            }

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_BEND_RIGHT_STRAIGHT;
              driveTimer = millis() + TIMER_DRIVE_BEND_RIGHT_STRAIGHT;
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_STRAIGHT:
            motorLeft.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveTimer < millis())
            {
              driveState = STATE_DRIVE_FORWARDS;
            }
            break;
        }
      } 
    }
  }
  else if(overallState == STATE_OVERALL_ALIGN_GOAL)
  {
    motorBrushes.stop();
    CAM_direction = camera.read();

    switch(goalState)
    {
      case STATE_GOAL_DRIVE_OVER_MAT:
        motorLeft.driveBackwards(MOTOR_LEFT_GOAL_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_GOAL_SPEED);

        if(goalTimer < millis() || (US_back_cm > 0 && US_back_cm < US_BACK_GOAL_MIN_DISTANCE))
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
          motorLeft.driveForwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_GOAL_SPEED);

          if(alignedToGoal() || goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_STOP; 
          }
          else if(CAM_direction == BEACON_RIGHT)
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
          }
          break;

        case STATE_GOAL_ROTATE_RIGHT:
          motorLeft.driveBackwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_GOAL_SPEED);

          if(alignedToGoal() || goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_STOP; 
          }
          else if(CAM_direction == BEACON_LEFT)
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          break;

        case STATE_GOAL_ROTATE_STOP:
          motorLeft.stop();
          motorRight.stop();

          if(goalTimer < millis())
          {
            if(alignedToGoal())
            {
              goalState = STATE_GOAL_BACKOFF;
              goalTimer = millis() + TIMER_GOAL_BACKOFF; 
            }
            else
            {
              goalState = STATE_GOAL_DRIVE_OVER_MAT; // Go back and check that we are aligned right.
              goalTimer = 0;
            }
          }
          break;

        case STATE_GOAL_BACKOFF:
          motorLeft.driveForwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_GOAL_SPEED);

          if(!(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON) || goalTimer < millis())
          {
            if(alignedToGoal())
            {
              goalState = STATE_GOAL_KICK_DELAY;
              goalTimer = millis() + TIMER_GOAL_KICK_DELAY;
            }
            else
            {
              goalState = STATE_GOAL_DRIVE_OVER_MAT; // Go back and check that we are aligned right.
              goalTimer = 0;						 //  Do not drive onto the mat in the process.
            }

            motorLeft.stop();   // This is to increase reaction speed.
            motorRight.stop();
          }
          break;

        case STATE_GOAL_KICK_DELAY:
          motorLeft.stop();
          motorRight.stop();

          if(goalTimer < millis())
          {
            servoTimer = millis() + TIMER_SERVO_KICK_1_DELAY;
            servoState = STATE_SERVO_KICK_1_DELAY;
            goalState = STATE_GOAL_KICK;
          }
          break;

        case STATE_GOAL_KICK:
          motorLeft.stop();
          motorRight.stop();
          break;

    }
  }
  else if(overallState == STATE_OVERALL_AVOID_GOAL)
  { 		// Could be done better by locking for camera, then driving away.
    switch(driveState)
    {
      case STATE_DRIVE_BACKOFF_LEFT_BACK:
        motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

        if(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON)
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_BACKOFF_LEFT_LEFT;
          driveTimer = TIMER_DRIVE_BACKOFF_LEFT_LEFT;		// This time is very short.
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_BACK:
        motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

        if(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON)
          driveTimer = millis() + TIMER_DRIVE_BACKOFF_LEFT_BACK;

        if(driveTimer < millis())
        {
          driveState = STATE_DRIVE_BACKOFF_RIGHT_RIGHT;
          driveTimer = TIMER_DRIVE_BACKOFF_RIGHT_RIGHT;
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_LEFT:
        motorLeft.driveBackwards(MOTOR_LEFT_NORMAL_SPEED);
        motorRight.driveForwards(MOTOR_RIGHT_NORMAL_SPEED);

        if(driveTimer < millis())
        {
          overallState = STATE_OVERALL_SEARCH_BALL;
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
        motorLeft.driveForwards(MOTOR_LEFT_NORMAL_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_NORMAL_SPEED);

        if(driveTimer < millis())
        {
          overallState = STATE_OVERALL_SEARCH_BALL;
          driveState = STATE_DRIVE_FORWARDS;
        }
        break;
    }
  }

  #ifdef PLOT_PRINT_MOTORS_ON  
    PLOT("driveTimer", driveTimer);
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
      else if(ballType == BALL_NONE)
      {
        servoState = STATE_SERVO_SERACH;
        overallState = STATE_OVERALL_SEARCH_BALL;
        driveState = STATE_DRIVE_FORWARDS;
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
        overallState = STATE_OVERALL_SEARCH_BALL;
        driveState = STATE_DRIVE_FORWARDS;
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
        servoState = STATE_SERVO_WRONG_BALL;
      }
      else if(ballType == BALL_NONE)
      {
        servoState = STATE_SERVO_SERACH;
        overallState = STATE_OVERALL_SEARCH_BALL;
        driveState = STATE_DRIVE_FORWARDS;
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
    PLOT("servoTimer", servoTimer);
  #endif
}

int determineBallType()
{
  int tempBallColour;
  int tempBallType;

  if(BALL.getColour2() > BALL_IR_Thresh)
  {
    if(BALL.getColour1() > BALL_RED_Thresh)
      tempBallColour = BALL_RED;
    else
      tempBallColour = BALL_BLUE;
  }
  else
  {
    tempBallColour = BALL_NONE;
  }

  if(tempBallColour == BALL_NONE)
    tempBallType = BALL_NONE;
  else if(tempBallColour == DESIRED_BALL_COLOUR)
    tempBallType = BALL_RIGHT;
  else
    tempBallType = BALL_WRONG;

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("tempBallType", tempBallType);
    PLOT("tempBallColour", tempBallColour);
  #endif  

  return tempBallType;
}

int determineGMLState()
{
  int tempGreenMatLeftState;

  if(GML.getColour2() < GML_GREEN_Thres && GML.getColour1() < GML_RED_Thres)
  {
    tempGreenMatLeftState = GREEN_MAT_ON;
  }
  else
  {
    tempGreenMatLeftState = GREEN_MAT_OFF;
  }

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("tempGreenMatLeftState", tempGreenMatLeftState);
  #endif  

  return tempGreenMatLeftState;
}

int determineGMRState()
{
  int tempGreenMatRightState;

  if(GMR.getColour2() < GMR_GREEN_Thres && GMR.getColour1() < GMR_RED_Thres)
  {
    tempGreenMatRightState = GREEN_MAT_ON;
  }
  else
  {
    tempGreenMatRightState = GREEN_MAT_OFF;
  }

  #ifdef PLOT_PRINT_COLOUR_ON
    PLOT("tempGreenMatRightState", tempGreenMatRightState);
  #endif  

  return tempGreenMatRightState;
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

bool alignedToGoal()
{	// May need to check if peak is centered more accuratly.
  return (CAM_direction == BEACON_CENTER && US_back_cm > 0 && US_back_cm < 100);
}