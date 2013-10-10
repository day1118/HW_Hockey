/*** Define overall states ***/
#define STATE_OVERALL_SEARCH_BALL       0
#define STATE_OVERALL_SEARCH_GOAL       1

/*** Define drive states ***/
#define	STATE_DRIVE_FORWARDS            0
#define STATE_DRIVE_BACKWARDS           1
#define STATE_DRIVE_BACKOFF_LEFT_BACK   2
#define STATE_DRIVE_BACKOFF_LEFT_LEFT   3
#define STATE_DRIVE_BACKOFF_RIGHT_BACK  4
#define STATE_DRIVE_BACKOFF_RIGHT_RIGHT 5
#define STATE_DRIVE_STOP                6

#define FORWARDS                        0
#define BACKWARDS                       1
#define STOP                            2

#define TIMER_DRIVE_BACKOFF_LEFT_BACK   1000
#define TIMER_DRIVE_BACKOFF_LEFT_LEFT   200
#define TIMER_DRIVE_STOP                100

#define TIMER_DRIVE_BACKOFF_RIGHT_BACK   TIMER_DRIVE_BACKOFF_LEFT_BACK
#define TIMER_DRIVE_BACKOFF_RIGHT_RIGHT  TIMER_DRIVE_BACKOFF_LEFT_LEFT