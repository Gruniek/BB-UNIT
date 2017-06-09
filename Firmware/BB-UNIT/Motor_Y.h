#include <Arduino.h>

//======================================================================
// Y88b   d88P
//  Y88b d88P
//   Y88o88P
//    Y888P
//     888
//     888
//     888
//     888
//======================================================================
    
void MOTOR_Y(int16_t G_Y, int SP_Y )
{
	bool    INVERT_Y       = false; // INVERT THE X MOTOR DIRECTION
	bool 	RUN_Y          = false;
	bool 	DIRECT_Y       = false;
	int     MIN_Y          = 45;    // SET THE MIN INCLINAISON FOR X
	int     MAX_Y          = 135;   // SET THE MAX INCLINAISON FOR X
	int     ETALON_Y       = 0;     // AJUSTEMENT FOR X
	int     ETALON_B_Y     = 0;     // AJUSTEMENT FOR X
	int     HYSTERESIS     = 1;     // HYSTERESSIS FOR THE ANGLE CALCULATION
	int     SPEED_Y        = 0;
	int     SETPOINT_Y     = 90;
	int     vy             = 0 ;
	int     MULTI          = 15;
	unsigned long previousMillisY = 0;



    //digitalWrite(PIN_ENABLE_XY, ENABLE_MOTOR_XY);
    // Direction
    if(G_Y < 90) DIRECT_Y = false;
    else DIRECT_Y = true;

    if(INVERT_Y)
    {
      if(DIRECT_Y) DIRECT_Y = false;
      else DIRECT_Y = true;
    }
    digitalWrite(PIN_DIRECTION_Y, DIRECT_Y); // dirX

    // Si ont doit demarer le moteur
    if(G_Y > (SP_Y + HYSTERESIS) || (G_Y < SP_Y - HYSTERESIS ))
    {
   
      if(G_Y > MIN_Y && G_Y < MAX_Y) RUN_Y = true;
      else RUN_Y = false;
    }
    else RUN_Y = false;

    // Calcul du PID Y
    if(G_Y < SP_Y) vy = (SP_Y - G_Y);
    if(G_Y == 0) vy = 0;
    if(G_Y > SETPOINT_Y) vy = (G_Y - SETPOINT_Y);

    SPEED_Y = 1000 / (vy);
    SPEED_Y = SPEED_Y * MULTI; // DEL
    if(SPEED_Y < 15) SPEED_Y = 15;

    // Mise en route du moteur
    unsigned long currentY = millis();
	//RUN_X = true;

    if (currentY - previousMillisY >= SPEED_Y)
    {
      previousMillisY = currentY;

      if(RUN_Y)
      {
        //Serial.println("stepY");
        digitalWrite(PIN_STEP_Y, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_Y, 0);
        delayMicroseconds( 500 );
      }
    }
}
