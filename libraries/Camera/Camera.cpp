#include "Camera.h"

Camera::Camera(String name, int pinSI, int pinCLK, int pinAIN){
	_pinSI = pinSI;
	_pinCLK = pinCLK;
	_pinAIN = pinAIN;
	_name = name;

	pinMode(_pinSI, OUTPUT);
	pinMode(_pinCLK, OUTPUT);
	pinMode(_pinAIN, INPUT);
}

int Camera::read(){
  int sensorValue;
  int CAM_startPoint, CAM_conseq, CAM_bestStart, CAM_bestWidth, CAM_center, CAM_direction;

  /*** Clear current values from camera ***/
  digitalWrite(CAMERA_SI_PIN, HIGH);
  digitalWrite(CAMERA_CLK_PIN, HIGH);

  digitalWrite(CAMERA_SI_PIN, LOW);	
  digitalWrite(CAMERA_CLK_PIN, LOW);

  for(int i = 0; i < CAMERA_RESOLUTION; i ++)
  {
    digitalWrite(CAMERA_CLK_PIN, HIGH);
    digitalWrite(CAMERA_CLK_PIN, LOW);
  }
  
	/****************************************/

  delayMicroseconds(CAMERA_EXPOSURE_TIME);      // wait for light to hit array
  
  CAM_startPoint = 0;
  CAM_conseq = 0;
  CAM_bestStart = 0;
  CAM_bestWidth = 0;

  #ifdef PLOT_PRINT_CAMERA_ON_DETAIL
    PLOT_PRINT("CAM_RAW");
  #endif

  digitalWrite(CAMERA_SI_PIN, HIGH);
  digitalWrite(CAMERA_CLK_PIN, HIGH);
  digitalWrite(CAMERA_SI_PIN, LOW);
  
  for(int i = 0; i < CAMERA_RESOLUTION; i ++)
  {
    digitalWrite(CAMERA_CLK_PIN, HIGH);
    sensorValue = analogRead(CAMERA_ANALOG_IN_PIN);
    #ifdef PLOT_PRINT_CAMERA_ON_DETAIL
      PLOT_PRINT(":");
      PLOT_PRINT(sensorValue);
    #endif

    digitalWrite(CAMERA_CLK_PIN, LOW);

    if(sensorValue > CAMERA__VALUE_Thresh)
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
 
 #ifdef PLOT_PRINT_CAMERA_ON_DETAIL
    PLOT_PRINTLN("");
  #endif

  if(CAM_bestWidth > CAMERA_MIN_WIDTH)
  {
    CAM_center = CAM_bestStart + (CAM_bestWidth/2);

    if(CAM_center < ((CAMERA_RESOLUTION/2) + CAMERA_CENTERED_WIDTH) && CAM_center > ((CAMERA_RESOLUTION/2) - CAMERA_CENTERED_WIDTH))
      CAM_direction = BEACON_CENTER;
    else if(CAM_center < (CAMERA_RESOLUTION/2))
      CAM_direction = BEACON_LEFT;
    else if(CAM_center > (CAMERA_RESOLUTION/2))
      CAM_direction = BEACON_RIGHT;
  }
  else
  {
    CAM_center = -1;
    CAM_direction = BEACON_NONE;
  }

  #if defined(PLOT_PRINT_CAMERA_ON) || defined(PLOT_PRINT_CAMERA_ON_DETAIL)
    PLOT("CAM_Start", CAM_bestStart);
    PLOT("CAM_Width", CAM_bestWidth);
    PLOT("CAM_direction", CAM_direction);
  #endif

	return CAM_direction;
}
