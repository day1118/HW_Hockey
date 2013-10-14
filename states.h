/*** Define overall states ***/
#define STATE_OVERALL_SEARCH_BALL       0
#define STATE_OVERALL_SEARCH_GOAL       1
#define STATE_OVERALL_ALIGN_GOAL		2

/*** Define drive states ***/
#define STATE_DRIVE_FORWARDS            0
#define STATE_DRIVE_BACKWARDS           1
#define STATE_DRIVE_BACKOFF_LEFT_BACK   2
#define STATE_DRIVE_BACKOFF_LEFT_LEFT   3
#define STATE_DRIVE_BACKOFF_RIGHT_BACK  4
#define STATE_DRIVE_BACKOFF_RIGHT_RIGHT 5
#define STATE_DRIVE_STOP                6
#define STATE_DRIVE_BEND_LEFT           7
#define STATE_DRIVE_BEND_RIGHT          8

#define FORWARDS                        0
#define BACKWARDS                       1
#define STOP                            2

#define TIMER_DRIVE_BACKOFF_LEFT_BACK   1000
#define TIMER_DRIVE_BACKOFF_LEFT_LEFT   200
#define TIMER_DRIVE_STOP                100
#define TIMER_DRIVE_BEND_LEFT           5000
#define TIMER_DRIVE_BEND_RIGHT          5000

#define TIMER_DRIVE_BACKOFF_RIGHT_BACK   TIMER_DRIVE_BACKOFF_LEFT_BACK
#define TIMER_DRIVE_BACKOFF_RIGHT_RIGHT  TIMER_DRIVE_BACKOFF_LEFT_LEFT

/*** Define servo states ***/
#define STATE_SERVO_SERACH				0
#define STATE_SERVO_WRONG_BALL			1
#define STATE_SERVO_RIGHT_BALL_DELAY	3
#define STATE_SERVO_RIGHT_BALL			4
#define STATE_SERVO_KICK_1_DELAY		5
#define STATE_SERVO_KICK_1				6
#define STATE_SERVO_KICK_2				7
#define STATE_SERVO_KICK_3				8

#define TIMER_SERVO_RIGHT_BALL   		1000
#define TIMER_SERVO_WRONG_BALL   		2000
#define TIMER_SERVO_KICK_1_DELAY		2000
#define TIMER_SERVO_KICK_1				2000
#define TIMER_SERVO_KICK_2				2000
#define TIMER_SERVO_KICK_3				2000

/*** Define goal states ***/
#define STATE_GOAL_DRIVE_OVER_MAT		0
#define STATE_GOAL_ROTATE_LEFT			1
#define STATE_GOAL_ROTATE_RIGHT			2
#define STATE_GOAL_BACKOFF				3
#define STATE_GOAL_KICK					4

#define TIMER_GOAL_DRIVE_OVER_MAT		1000
#define TIMER_GOAL_ROTATE_LEFT			10000
#define TIMER_GOAL_ROTATE_RIGHT			10000
#define TIMER_GOAL_BACKOFF				2000

#define BALL_NONE						0
#define BALL_RIGHT						1
#define BALL_WRONG						2
#define BALL_RED						1
#define BALL_BLUE						2

#define GREEN_MAT_OFF					0
#define GREEN_MAT_ON					1

#define BEACON_NONE						0
#define BEACON_LEFT						1
#define BEACON_RIGHT					2
#define BEACON_CENTER					3