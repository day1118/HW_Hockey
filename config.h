/*** Define debug level ***/
#define	DEBUG_PRINT_OFF
#define	PLOT_PRINT_ON

#define MOTORS_ON  

#define PLOT_PRINT_STATUS_ON
#define PLOT_PRINT_TOUCH_OFF
#define PLOT_PRINT_IR_OFF
#define PLOT_PRINT_COLOUR_ON
#define PLOT_PRINT_SERVOS_OFF
#define PLOT_PRINT_MOTORS_OFF
#define PLOT_PRINT_CAMERA_ON
#define PLOT_PRINT_US_ON

#define LED_READ_DELAY_TIME                 4
#define CAMERA_DELAY_TIME                   20

#define DESIRED_BALL_COLOUR                 BALL_RED

/*** Define named positions ***/
#define SERVO_FRONT_UP                      75
#define SERVO_FRONT_DOWN                    5

#define SERVO_BACK_UP                       110
#define SERVO_BACK_DOWN                     180

#define SERVO_KICK_UP                       90
#define SERVO_KICK_DOWN                     10

#define MAX_ULTRASONIC_DISTANCE_CM          100

/*** Define constants ***/
#define SERVO_FRONT_MIN_PWM                 450
#define SERVO_FRONT_MAX_PWM                 2450
#define SERVO_BACK_MIN_PWM                  450
#define SERVO_BACK_MAX_PWM                  2450
#define SERVO_KICK_MIN_PWM                  800
#define SERVO_KICK_MAX_PWM                  2200

/*** Define pinouts ***/
#define FRONT_ULTRASONIC_SENSOR_TRIG        31
#define FRONT_ULTRASONIC_SENSOR_ECHO        33

#define IRFL_IR_LED_PIN                     32
#define IRFL_SIDE_PHOTOTRANSISTOR_PIN       A10
#define IRFL_FRONT_PHOTOTRANSISTOR_PIN      A9
  
#define GREEN_MAT_LEFT_RED_LED_PIN          24
#define GREEN_MAT_LEFT_GREEN_LED_PIN        26
#define GREEN_MAT_LEFT_PHOTOTRANSISTOR_PIN  A12

#define BALL_COLOUR_RED_LED_PIN             28
#define BALL_COLOUR_IR_LED_PIN              30
#define BALL_COLOUR_PHOTOTRANSISTOR_PIN     A11

#define SERVO_KICK_PIN			    		        4
#define SERVO_FRONT_PIN			   	 		        2
#define SERVO_BACK_PIN			   		 	        3

#define IRBL_IR_LED_PIN                     25
#define IRBL_SIDE_PHOTOTRANSISTOR_PIN       A15
#define IRBL_BACK_PHOTOTRANSISTOR_PIN       A14

#define CAMERA_ANALOG_IN_PIN                A13
#define CAMERA_CLK_PIN                      29
#define CAMERA_SI_PIN                       27

#define FRONT_LEFT_TOUCH_SENSOR             22
#define BACK_LEFT_TOUCH_SENSOR              23
#define FRONT_RIGHT_TOUCH_SENSOR            42
#define BACK_RIGHT_TOUCH_SENSOR             43

#define IRFR_IR_LED_PIN                     46
#define IRFR_SIDE_PHOTOTRANSISTOR_PIN       A4
#define IRFR_FRONT_PHOTOTRANSISTOR_PIN      A5

#define BACK_ULTRASONIC_SENSOR_TRIG         48
#define BACK_ULTRASONIC_SENSOR_ECHO         44

#define GREEN_MAT_RIGHT_RED_LED_PIN         45
#define GREEN_MAT_RIGHT_GREEN_LED_PIN       49
#define GREEN_MAT_RIGHT_PHOTOTRANSISTOR_PIN A8

#define IRBR_IR_LED_PIN                     47
#define IRBR_SIDE_PHOTOTRANSISTOR_PIN       A6
#define IRBR_BACK_PHOTOTRANSISTOR_PIN       A7

#define MOTOR_L_A_PIN		            	35
#define MOTOR_L_B_PIN		            	34
#define MOTOR_L_ENABLE_PIN		    		7

#define MOTOR_R_A_PIN		            	37
#define MOTOR_R_B_PIN		            	36
#define MOTOR_R_ENABLE_PIN		    		6

#define MOTOR_B_A_PIN		            	39
#define MOTOR_B_B_PIN		            	38
#define MOTOR_B_ENABLE_PIN		    		5

#define ENCODER_LEFT_PIN					A2
#define ENCODER_RIGHT_PIN					A3

/*** Define print statements ***/
#ifdef DEBUG_PRINT_ON
  #define DEBUG_PRINT(x)        Serial.print (x)
  #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
  #define DEBUG_PRINTLN(x)      Serial.println (x)
#else
  #define DEBUG_PRINT(x)        
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x) 
#endif

#ifdef PLOT_PRINT_ON
  #define PLOT_PRINT(x)        Serial.print (x)
  #define PLOT_PRINTDEC(x)     Serial.print (x, DEC)
  #define PLOT_PRINTLN(x)      Serial.println (x)
  #define PLOT(x, y)           PLOT_PRINT(x); PLOT_PRINT(':'); PLOT_PRINTLN(y);
#else
  #define PLOT_PRINT(x)
  #define PLOT_PRINTDEC(x)
  #define PLOT_PRINTLN(x)
  #define PLOT(x, y) 
#endif

#define PLOT_OVERRIDE(x, y) Serial.print (x); Serial.print(':'); Serial.println(y);
