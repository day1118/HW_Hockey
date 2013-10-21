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
  #include "StateMachine.h"

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

StateMachine overallState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
StateMachine driveState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
StateMachine stalledState(STATE_NOT_STALLED, NEVER_EXPIRE);
int servoState = STATE_SERVO_SERACH;
int goalState = STATE_GOAL_DRIVE_OVER_MAT_LEFT;

int servoFPos, servoBPos, servoKPos;
int ballType = BALL_NONE;
int greenMatLeftState = GREEN_MAT_OFF;
int greenMatRightState = GREEN_MAT_OFF;
int CAM_direction;

bool wallFollowLeft = true;

unsigned long servoTimer = 0, goalTimer = 0;

unsigned int US_back_cm;
unsigned int US_front_cm;

int goalAlignRotateAttempts = 0;

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
  readColourSensors();
  readUSSensors();
  setMotors();
  setServos();
  stallDetected();

  #if defined PLOT_PRINT_CAMERA_ON_DETAIL && defined PLOT_PRINT_ON
    camera.read();
  #endif

  #ifdef PLOT_PRINT_STATUS_ON
    PLOT("overallState", overallState.getState());
    PLOT("overallStateTime", overallState.getTimeSinceChange());
    PLOT("wallFollowLeft", wallFollowLeft);
    PLOT("driveState", driveState.getState());
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
  if(overallState.getState() == STATE_OVERALL_SEARCH_BALL)
  {
    motorBrushes.driveForwards(MOTOR_BRUSHES_NORMAL_SPEED);

    if(greenMatRightState == GREEN_MAT_ON || greenMatLeftState == GREEN_MAT_ON)
    {
      //TODO - Handle avoiding goal.
      overallState.setState(STATE_OVERALL_AVOID_GOAL, NEVER_EXPIRE);
      if(wallFollowLeft)  
        driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
      else
        driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
    }
    else
    {
      #ifdef AUTO_REVERSE_DIRECTION
        if(overallState.expired())  // If time is up, reverse direction.
        {
          wallFollowLeft = !wallFollowLeft;
          overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
          if(wallFollowLeft)
            driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
          else
            driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
        }
      #endif

      if(wallFollowLeft)
      {
        if(stallDetected())
        {
          driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
        }
        else if(TFL.on() || TFR.on())
        {   // Touch sensors or front is very close
          driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
        }

        switch(driveState.getState())
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED + MOTOR_LEFT_FORWARD_LEFT_BEND_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED + MOTOR_RIGHT_FORWARD_LEFT_BEND_SPEED);

            if(TFL.on() || TFR.on() || IRFL.getFront() > IRFL_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
            }
            else if(IRFL.getFront() > IRFL_FRONT_FAR_Thresh && !(IRFL.getSide() > IRFL_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_RIGHT, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_RIGHT);
            } 
            else if(IRFL.getSide() > IRFL_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState.setState(STATE_DRIVE_BEND_RIGHT_RIGHT, TIMER_DRIVE_FORWARD_BEND_RIGHT_RIGHT);
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_BACK:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(driveState.expired() || !(IRFL.getFront() > IRFL_FRONT_FAR_Thresh || IRFR.getFront() > IRFR_FRONT_FAR_Thresh || IRFL.getSide() > IRFL_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_RIGHT, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_RIGHT);
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            //motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);
            motorRight.stop();

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_RIGHT:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.stop();

            if(IRBR.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)
            { // Stalled for a long time, try again.
              // Try doing wabble again.
              if(driveState.expired())
              {
                driveState.setState(STATE_DRIVE_BEND_RIGHT_STRAIGHT, TIMER_DRIVE_FORWARD_BEND_RIGHT_STRAIGHT);
              }
            }
            else if(IRFL.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_BACK);
            }
            else if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_BEND_RIGHT_STRAIGHT, TIMER_DRIVE_FORWARD_BEND_RIGHT_STRAIGHT);
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_STRAIGHT:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          default:
            driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
      }
      else
      {
        if(stallDetected())
        {
          driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
        }
        else if(TFL.on() || TFR.on() || IRFR.getFront() > IRFR_FRONT_CLOSE_Thresh)
        {   // Touch sensors or front is very close
          driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
        } 

        switch(driveState.getState())
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED + MOTOR_LEFT_FORWARD_RIGHT_BEND_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED + MOTOR_RIGHT_FORWARD_RIGHT_BEND_SPEED);

            if(TFL.on() || TFR.on() || IRFR.getFront() > IRFR_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
            } 
            else if(IRFR.getFront() > IRFR_FRONT_FAR_Thresh && !(IRFR.getSide() > IRFR_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_LEFT, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_LEFT);
            } 
            else if(IRFR.getSide() > IRFR_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState.setState(STATE_DRIVE_BEND_LEFT_LEFT, TIMER_DRIVE_FORWARD_BEND_LEFT_LEFT);
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_BACK:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(driveState.expired() || !(IRFR.getFront() > IRFR_FRONT_FAR_Thresh || IRFR.getFront() > IRFR_FRONT_FAR_Thresh  || IRFR.getSide() > IRFR_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_LEFT, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_LEFT);
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_LEFT:
            //motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorLeft.stop();
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_BEND_LEFT_LEFT:
            motorLeft.stop();
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(IRFR.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)  // TODO: Think about this
            { // Stalled for a long time, try again.
              // Try doing wabble again.
             if(driveState.expired())
              {
                driveState.setState(STATE_DRIVE_BEND_RIGHT_STRAIGHT, TIMER_DRIVE_FORWARD_BEND_RIGHT_STRAIGHT);
              }
            }
            else if(IRFR.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_BACK);
            } else if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_BEND_LEFT_STRAIGHT, TIMER_DRIVE_FORWARD_BEND_LEFT_STRAIGHT);
            }
            break;

          case STATE_DRIVE_BEND_LEFT_STRAIGHT:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          default:
            driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
      } 
    }
  }
  else if(overallState.getState() == STATE_OVERALL_SEARCH_GOAL)
  {
    motorBrushes.driveBackwards(MOTOR_BRUSHES_NORMAL_SPEED);

    if(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON)
    {
      overallState.setState(STATE_OVERALL_ALIGN_GOAL, TIMER_OVERALL_ALIGN_GOAL);
      goalTimer = millis() + TIMER_GOAL_DRIVE_OVER_MAT;
      if(greenMatLeftState == GREEN_MAT_ON)
        goalState = STATE_GOAL_DRIVE_OVER_MAT_RIGHT;
      else
        goalState = STATE_GOAL_DRIVE_OVER_MAT_LEFT;
    }
    else
    {
      #ifdef AUTO_REVERSE_DIRECTION
        if(overallState.expired())  // If time is up, reverse direction.
        {
          wallFollowLeft = !wallFollowLeft;
          overallState.setState(STATE_OVERALL_SEARCH_GOAL, TIMER_OVERALL_SEARCH_GOAL);
          if(wallFollowLeft)
            driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_BACK);
          else
            driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_BACK);
        }
      #endif
      
      if(wallFollowLeft)
      {
        if(TBL.on() || TBR.on())
        {   // Touch sensors or front is very close
          driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_BACK);
        }

        switch(driveState.getState())
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED + MOTOR_LEFT_BACKWARD_LEFT_BEND_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED + MOTOR_RIGHT_BACKWARD_LEFT_BEND_SPEED);

            if(TBL.on() || TBR.on() || IRBR.getFront() > IRBR_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_BACK);
            }
            else if(IRBR.getFront() > IRBR_FRONT_FAR_Thresh && !(IRBR.getSide() > IRBR_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_RIGHT, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_RIGHT);
            }
            else
             if(IRBR.getSide() > IRBR_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState.setState(STATE_DRIVE_BEND_RIGHT_RIGHT, TIMER_DRIVE_BACKWARD_BEND_RIGHT_RIGHT);
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_BACK:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired() || !(IRBL.getFront() > IRBL_FRONT_FAR_Thresh || IRBR.getFront() > IRBR_FRONT_FAR_Thresh || IRBR.getSide() > IRBR_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_RIGHT, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_RIGHT);
            }
            break;

          case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_RIGHT:
            motorLeft.stop();
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(IRBR.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)
            { // Stalled for a long time, try again.
              // Try doing wabble again.
            }
            else if(IRBR.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_RIGHT_BACK);
            }
            else if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_BEND_RIGHT_STRAIGHT, TIMER_DRIVE_BACKWARD_BEND_RIGHT_STRAIGHT);
            }
            break;

          case STATE_DRIVE_BEND_RIGHT_STRAIGHT:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          default:
            driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
      }
      else
      {
        if(TBL.on() || TBR.on())
        {   // Touch sensors or front is very close
          driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_BACK);
        }

        switch(driveState.getState())
        {
          case STATE_DRIVE_FORWARDS:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED  + MOTOR_LEFT_BACKWARD_RIGHT_BEND_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED + MOTOR_RIGHT_BACKWARD_RIGHT_BEND_SPEED);

            if(TBL.on() || TBR.on() || IRBL.getFront() > IRBL_FRONT_CLOSE_Thresh)
            {   // Touch sensors or front is very close
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_BACK);
            }
            else if(IRBL.getFront() > IRBL_FRONT_FAR_Thresh && !(IRBL.getSide() > IRBL_SIDE_FAR_Thresh))
            {   // Something in front, but not beside
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_LEFT, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_LEFT);
            }
            else
             if(IRBL.getSide() > IRBL_SIDE_CLOSE_Thresh)
            {   // Something beside
              driveState.setState(STATE_DRIVE_BEND_LEFT_LEFT, TIMER_DRIVE_BACKWARD_BEND_LEFT_LEFT);
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_BACK:
            motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired() || !(IRBL.getFront() > IRBL_FRONT_FAR_Thresh || IRBR.getFront() > IRBR_FRONT_FAR_Thresh  || IRBL.getSide() > IRBL_SIDE_CLOSE_Thresh))
            { // If time is up, or nothing in front
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_LEFT, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_LEFT);
            }
            break;

          case STATE_DRIVE_BACKOFF_LEFT_LEFT:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_BEND_LEFT_LEFT:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.stop();

            if(IRBL.sideGetTimeSinceChange() > 2 * CORNER_STALL_DETECT_TIME)  // TODO: Think about this
            { // Stalled for a long time, try again.
              // Try doing wabble again.
            }
            else if(IRBL.sideGetTimeSinceChange() > CORNER_STALL_DETECT_TIME)
            { // Stalled. Backoff.
              driveState.setState(STATE_DRIVE_BACKOFF_LEFT_BACK, TIMER_DRIVE_BACKWARD_BACKOFF_LEFT_BACK);
            } else if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_BEND_LEFT_STRAIGHT, TIMER_DRIVE_BACKWARD_BEND_LEFT_STRAIGHT);
            }
            break;

          case STATE_DRIVE_BEND_LEFT_STRAIGHT:
            motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
            motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          case STATE_DRIVE_STOP:
            motorLeft.stop();
            motorRight.stop();

            if(driveState.expired())
            {
              driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
            }
            break;

          default:
            driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
      } 
    }
  }
  else if(overallState.getState() == STATE_OVERALL_ALIGN_GOAL)
  {
    motorBrushes.stop();
    CAM_direction = camera.read();

    if(overallState.expired() && !(goalState == STATE_GOAL_KICK || goalState == STATE_GOAL_KICK_DELAY))
    {
      overallState.setState(STATE_OVERALL_AVOID_GOAL);
    }
    else
    {
      switch(goalState)
      {
        case STATE_GOAL_DRIVE_OVER_MAT_LEFT:
          motorLeft.driveBackwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_GOAL_SPEED);

          if(goalTimer < millis() || (US_back_cm > 0 && US_back_cm < US_BACK_GOAL_MIN_DISTANCE))
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalAlignRotateAttempts = 0;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          break;

        case STATE_GOAL_DRIVE_OVER_MAT_RIGHT:
          motorLeft.driveBackwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_GOAL_SPEED);

          if(goalTimer < millis() || (US_back_cm > 0 && US_back_cm < US_BACK_GOAL_MIN_DISTANCE))
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalAlignRotateAttempts = 0;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
          }
          break;

        case STATE_GOAL_ROTATE_LEFT:
          motorLeft.driveBackwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_GOAL_SPEED);

          if(alignedToGoal())   // TODO - Could stop when it is to the left, then adjust.
          {
            goalState = STATE_GOAL_ROTATE_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_STOP; 
          }
          else if((CAM_direction == BEACON_LEFT && beaconDetected()) || (goalAlignRotateAttempts * (TIMER_GOAL_ROTATE_LEFT + TIMER_GOAL_ROTATE_LEFT_STOP)) > GOAL_ROTATE_TIME_MAX)
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalAlignRotateAttempts = 0;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
          } 
          else if(goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_LEFT_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT_STOP;
            goalAlignRotateAttempts ++;
          }
          break;

        case STATE_GOAL_ROTATE_LEFT_STOP:
          motorLeft.stop();
          motorRight.stop();

          if(goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          break;

        case STATE_GOAL_ROTATE_RIGHT:
          motorLeft.driveForwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveBackwards(MOTOR_RIGHT_GOAL_SPEED);

          if(alignedToGoal())   // TODO - Could stop when it is to the left, then adjust.
          {
            goalState = STATE_GOAL_ROTATE_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_STOP; 
          }
          else if((CAM_direction == BEACON_RIGHT && beaconDetected()) || (goalAlignRotateAttempts * (TIMER_GOAL_ROTATE_RIGHT + TIMER_GOAL_ROTATE_RIGHT_STOP)) > GOAL_ROTATE_TIME_MAX))
          {
            goalState = STATE_GOAL_ROTATE_LEFT;
            goalAlignRotateAttempts = 0;
            goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
          }
          else if(goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_RIGHT_STOP;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT_STOP;
            goalAlignRotateAttempts ++;
          }
          break;

        case STATE_GOAL_ROTATE_RIGHT_STOP:
          motorLeft.stop();
          motorRight.stop();

          if(goalTimer < millis())
          {
            goalState = STATE_GOAL_ROTATE_RIGHT;
            goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
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
              goalState = STATE_GOAL_DRIVE_OVER_MAT_LEFT; // Go back and check that we are aligned right.
              goalTimer = 0;
            }
          }
          break;

        case STATE_GOAL_BACKOFF:
          motorLeft.driveForwards(MOTOR_LEFT_GOAL_SPEED);
          motorRight.driveForwards(MOTOR_RIGHT_GOAL_SPEED);

          if(!(greenMatLeftState == GREEN_MAT_ON || greenMatRightState == GREEN_MAT_ON))
          {
            if(alignedToGoal())
            {
              goalState = STATE_GOAL_KICK_DELAY;
              goalTimer = millis() + TIMER_GOAL_KICK_DELAY;
            }
            else
            {
              if(CAM_direction == BEACON_RIGHT && beaconDetected())
              {
                goalState = STATE_GOAL_ROTATE_LEFT;
                goalAlignRotateAttempts = 0;
                goalTimer = millis() + TIMER_GOAL_ROTATE_LEFT;
              }
              else
              {
                goalState = STATE_GOAL_ROTATE_RIGHT;
                goalAlignRotateAttempts = 0;
                goalTimer = millis() + TIMER_GOAL_ROTATE_RIGHT;
              }
            }
          }
          else if(goalTimer < millis())
          {
            goalState = STATE_GOAL_BACKOFF_STOP;
            goalTimer = millis() + TIMER_GOAL_BACKOFF_STOP;
          }
          break;

        case STATE_GOAL_BACKOFF_STOP:
          motorLeft.stop();
          motorRight.stop();

          if(goalTimer < millis())
          {
            goalState = STATE_GOAL_BACKOFF;
            goalTimer = millis() + TIMER_GOAL_BACKOFF;
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
  }
  else if(overallState.getState() == STATE_OVERALL_AVOID_GOAL)
  { 		// Could be done better by locking for camera, then driving away.
    CAM_direction = camera.read();
    if(beaconDetected())
    {
      if(driveState.getState() != STATE_DRIVE_FORWARDS)
        driveState.setState(STATE_DRIVE_FORWARDS, STATE_DRIVE_BACKOFF_LEFT_BACK);
    }
    switch(driveState.getState())
    {
      case STATE_DRIVE_BACKOFF_LEFT_BACK:
        motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

        if(driveState.expired())
        {
          driveState.setState(STATE_DRIVE_BACKOFF_LEFT_LEFT, TIMER_DRIVE_FORWARD_BACKOFF_LEFT_LEFT);
        }
        break;

      case STATE_DRIVE_BACKOFF_LEFT_LEFT:
        motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
        motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

        if(driveState.expired())
        {
          unsigned long extraTime = overallState.getTimeSinceChange() + overallState.getTimeSinceChangePrev();
          overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
          overallState.addTimeSinceChange(extraTime);
          driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_BACK:
        motorLeft.driveBackwards(MOTOR_LEFT_BACKWARD_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

        if(driveState.expired())
        {
          driveState.setState(STATE_DRIVE_BACKOFF_RIGHT_RIGHT, TIMER_DRIVE_FORWARD_BACKOFF_RIGHT_RIGHT);
        }
        break;

      case STATE_DRIVE_BACKOFF_RIGHT_RIGHT:
        motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
        motorRight.driveBackwards(MOTOR_RIGHT_BACKWARD_SPEED);

        if(driveState.expired())
        {
          unsigned long extraTime = 0;
          if(overallState.getStatePrev == STATE_OVERALL_SEARCH_BALL)
            extraTime = overallState.getTimeSinceChange() + overallState.getTimeSinceChangePrev();
          overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
          overallState.addTimeSinceChange(extraTime);
          driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
        break;

      case STATE_DRIVE_FORWARDS:
        motorLeft.driveForwards(MOTOR_LEFT_FORWARD_SPEED);
        motorRight.driveForwards(MOTOR_RIGHT_FORWARD_SPEED);

        if(driveState.expired())
        {
          unsigned long extraTime = 0;
          if(overallState.getStatePrev == STATE_OVERALL_SEARCH_BALL)
            extraTime = overallState.getTimeSinceChange() + overallState.getTimeSinceChangePrev();
          overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
          overallState.addTimeSinceChange(extraTime);
          driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        }
        break;

    }
  }
  else
  {
    overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL); 
  }

  #ifdef PLOT_PRINT_MOTORS_ON  
    PLOT("driveTimer", driveState.getTimeSinceChange());
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
        overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
        driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
      }

      if(servoTimer < millis())
      {
        overallState.setState(STATE_OVERALL_SEARCH_GOAL, TIMER_OVERALL_SEARCH_GOAL);
        driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        servoState = STATE_SERVO_RIGHT_BALL;
      }
      break;
    
    case STATE_SERVO_RIGHT_BALL:
      servoFPos = SERVO_FRONT_DOWN;
      servoBPos = SERVO_BACK_DOWN;
      servoKPos = SERVO_KICK_UP;

      if(ballType == BALL_WRONG)
      {
        overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
        driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        servoTimer = millis() + TIMER_SERVO_WRONG_BALL;
        servoState = STATE_SERVO_WRONG_BALL;
      }
      else if(ballType == BALL_NONE)
      {
        servoState = STATE_SERVO_SERACH;
        overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
        driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
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
        overallState.setState(STATE_OVERALL_SEARCH_BALL, TIMER_OVERALL_SEARCH_BALL);
        driveState.setState(STATE_DRIVE_FORWARDS, NEVER_EXPIRE);
        servoState = STATE_SERVO_SERACH;
        clearStallDetect();
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
  return (CAM_direction == BEACON_CENTER && beaconDetected());
}

bool stallDetected()
{
  #ifdef STALL_DETECTION_ON
    int unchangedSensors = 0;
    int unchangedHighSensors = 0;
    if(IRFL.frontGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRFL.frontOn())
        unchangedHighSensors ++;
    }
    if(IRFL.sideGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRFL.sideOn())
        unchangedHighSensors ++;
    }
    if(IRFR.frontGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRFR.frontOn())
        unchangedHighSensors ++;
    }
    if(IRFR.sideGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRFR.sideOn())
        unchangedHighSensors ++;
    }
    if(IRBL.frontGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRBL.frontOn())
        unchangedHighSensors ++;
    }
    if(IRBL.sideGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRBL.sideOn())
        unchangedHighSensors ++;
    }
    if(IRBR.frontGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRBR.frontOn())
        unchangedHighSensors ++;
    }
    if(IRBR.sideGetTimeSinceChange() > STALL_TIME)
    {
      unchangedSensors ++;
      if(IRBR.sideOn())
        unchangedHighSensors ++;
    }

    bool stalled = (unchangedHighSensors >= 2 || unchangedSensors >= 6);
    
    if(stalled)
    {
      clearStallDetect();
    }

    #ifdef PLOT_PRINT_STATUS_ON
      PLOT("unchangedHighSensors", unchangedHighSensors);
      PLOT("unchangedSensors", unchangedSensors);
      PLOT("stalled", stalled);
    #endif

    return stalled;

  #else
    return false;
  #endif
}

void clearStallDetect()
{
  IRFL.frontResetTimeSinceChange();
  IRFL.sideResetTimeSinceChange();
  IRFR.frontResetTimeSinceChange();
  IRFR.sideResetTimeSinceChange();
  IRBL.frontResetTimeSinceChange();
  IRBL.sideResetTimeSinceChange();
  IRBR.frontResetTimeSinceChange();
  IRBR.sideResetTimeSinceChange();
}

bool beaconDetected()
{
  return (CAM_direction != BEACON_NONE && US_back_cm > 0 && US_back_cm < 100)
}