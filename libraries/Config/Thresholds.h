#define IRFL_FRONT_Thresh		200
#define IRFR_FRONT_Thresh		150
#define IRFL_SIDE_Thresh		200
#define IRFR_SIDE_Thresh		180

#define IRBL_FRONT_CLOSE_Thresh 	350
#define IRBL_FRONT_FAR_Thresh		100		// Coul be 150
#define IRBR_FRONT_CLOSE_Thresh 	200
#define IRBR_FRONT_FAR_Thresh		100
#define IRBL_SIDE_CLOSE_Thresh		250
#define IRBL_SIDE_FAR_Thresh		100
#define IRBR_SIDE_CLOSE_Thresh		300
#define IRBR_SIDE_FAR_Thresh		100

#define IRBL_BACK_Thresh		IRBL_FRONT_FAR_Thresh /// These should be de
#define IRBR_BACK_Thresh		IRBR_FRONT_FAR_Thresh
#define IRBL_SIDE_Thresh		IRBL_SIDE_FAR_Thresh /// These should be de
#define IRBR_SIDE_Thresh		IRBR_SIDE_FAR_Thresh

#define BALL_IR_Thresh          150
#define BALL_RED_Thresh         100

#define GML_GREEN_Thres			60	
#define GML_RED_Thres			200

#define GMR_GREEN_Thres			180
#define GMR_RED_Thres			200

#define GM_FILTER_SIZE			5

#define US_BACK_GOAL_MIN_DISTANCE	60

#define MOTOR_LEFT_NORMAL_SPEED	220
#define MOTOR_LEFT_GOAL_SPEED	220

#define MOTOR_RIGHT_NORMAL_SPEED	240
#define MOTOR_RIGHT_GOAL_SPEED	200

#define MOTOR_BRUSHES_NORMAL_SPEED	200

#define CAMERA__VALUE_Thresh		800
#define CAMERA_MIN_WIDTH			10
#define CAMERA_CENTERED_WIDTH		40

#define CORNER_STALL_DETECT_TIME	5000