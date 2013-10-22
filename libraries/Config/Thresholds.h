#define IRFL_FRONT_CLOSE_Thresh 	300	
#define IRFL_FRONT_FAR_Thresh		120	
#define IRFR_FRONT_CLOSE_Thresh 	300	
#define IRFR_FRONT_FAR_Thresh		120	
#define IRFL_SIDE_CLOSE_Thresh		500
#define IRFL_SIDE_FAR_Thresh		200
#define IRFR_SIDE_CLOSE_Thresh		500	
#define IRFR_SIDE_FAR_Thresh		200

/*#define IRFL_FRONT_Thresh			IRFL_FRONT_CLOSE_Thresh
#define IRFR_FRONT_Thresh			IRFR_FRONT_CLOSE_Thresh
#define IRFL_SIDE_Thresh			IRFL_SIDE_CLOSE_Thresh
#define IRFR_SIDE_Thresh			IRFR_SIDE_CLOSE_Thresh*/

#define IRBL_FRONT_CLOSE_Thresh 	350
#define IRBL_FRONT_FAR_Thresh		200	
#define IRBR_FRONT_CLOSE_Thresh 	200
#define IRBR_FRONT_FAR_Thresh		150	
#define IRBL_SIDE_CLOSE_Thresh		250
#define IRBL_SIDE_FAR_Thresh		100
#define IRBR_SIDE_CLOSE_Thresh		300	
#define IRBR_SIDE_FAR_Thresh		100

/*#define IRBL_BACK_Thresh		IRBL_FRONT_CLOSE_Thresh /// These should be de
#define IRBR_BACK_Thresh		IRBR_FRONT_CLOSE_Thresh
#define IRBL_SIDE_Thresh		IRBL_SIDE_CLOSE_Thresh /// These should be de
#define IRBR_SIDE_Thresh		IRBR_SIDE_CLOSE_Thresh*/

#define BALL_IR_Thresh          150
#define BALL_RED_Thresh         60

#define GML_GREEN_Thres			100
#define GML_RED_Thres			300
	
#define GMR_GREEN_Thres			140
#define GMR_RED_Thres			150

#define GM_FILTER_SIZE			5

#define US_BACK_GOAL_MIN_DISTANCE	60
#define US_FRONT_WALL_DISTANCE		30

#define MOTOR_LEFT_FORWARD_SPEED	255
#define MOTOR_RIGHT_FORWARD_SPEED	200

#define MOTOR_LEFT_FORWARD_LEFT_BEND_SPEED		0//-20
#define MOTOR_RIGHT_FORWARD_LEFT_BEND_SPEED		0//20
#define MOTOR_LEFT_FORWARD_RIGHT_BEND_SPEED		0
#define MOTOR_RIGHT_FORWARD_RIGHT_BEND_SPEED	0

#define MOTOR_LEFT_BACKWARD_SPEED	250
#define MOTOR_RIGHT_BACKWARD_SPEED	200

#define MOTOR_LEFT_BACKWARD_LEFT_BEND_SPEED		0
#define MOTOR_RIGHT_BACKWARD_LEFT_BEND_SPEED	MOTOR_LEFT_BACKWARD_LEFT_BEND_SPEED
#define MOTOR_LEFT_BACKWARD_RIGHT_BEND_SPEED	MOTOR_LEFT_BACKWARD_LEFT_BEND_SPEED
#define MOTOR_RIGHT_BACKWARD_RIGHT_BEND_SPEED	MOTOR_LEFT_BACKWARD_LEFT_BEND_SPEED


#define MOTOR_LEFT_GOAL_SPEED		MOTOR_LEFT_FORWARD_SPEED
#define MOTOR_RIGHT_GOAL_SPEED		MOTOR_RIGHT_FORWARD_SPEED

#define MOTOR_BRUSHES_NORMAL_SPEED	200

#define CAMERA__VALUE_Thresh		600
#define CAMERA_MIN_WIDTH			10
#define CAMERA_CENTERED_WIDTH		15

#define CORNER_STALL_DETECT_TIME	4000

#define STALL_TIME 					7000