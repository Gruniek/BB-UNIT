#include <Arduino.h>

//======================================================================
// Y88b   d88P
//  Y88b d88P
//   Y88o88P
//    Y888P
//    d888b
//   d88888b
//  d88P Y88b
// d88P   Y88b
//======================================================================

void MOTOR_X(int16_t G_X, int SP_X )
{
	bool    INVERT_X       = false; // INVERT THE X MOTOR DIRECTION
	bool 	RUN_X          = false;
	bool 	DIRECT_X       = false;
	int     MIN_X          = 45;    // SET THE MIN INCLINAISON FOR X
	int     MAX_X          = 135;   // SET THE MAX INCLINAISON FOR X
	int     ETALON_X       = 0;     // AJUSTEMENT FOR X
	int     ETALON_B_X     = 0;     // AJUSTEMENT FOR X
	int     HYSTERESIS     = 1;     // HYSTERESSIS FOR THE ANGLE CALCULATION
	int     SPEED_X        = 0;
	int     SETPOINT_X     = 90;
	int     vx             = 0 ;
	int     MULTI          = 15;
	unsigned long previousMillisX = 0;

    // Direction
    if(G_X < 90) DIRECT_X = false;
    else DIRECT_X = true;

    if(INVERT_X)
    {
      if(DIRECT_X) DIRECT_X = false;
      else DIRECT_X = true;
    }
    digitalWrite(PIN_DIRECTION_X, DIRECT_X); // dirX

    // Si ont doit demarer le moteur
    if(G_X > (SP_X + HYSTERESIS) || (G_X < SP_X - HYSTERESIS ))
    {
      
      if(G_X > MIN_X && G_X < MAX_X) RUN_X = true;
      else RUN_X = false;
    }
    else RUN_X = false;

    // Calcul du PID X
    if(G_X < SP_X) vx = (SP_X - G_X);
    if(G_X == 0) vx = 0;
    if(G_X > SETPOINT_X) vx = (G_X - SETPOINT_X);

    SPEED_X = 1000 / (vx);
    SPEED_X = SPEED_X * MULTI; // DEL
    if(SPEED_X < 15) SPEED_X = 15;

    // Mise en route du moteur
    unsigned long currentX = millis();
	//RUN_X = true;

    if (currentX - previousMillisX >= SPEED_X)
    {
      previousMillisX = currentX;

      if(RUN_X)
      {
        //Serial.println("stepX");
        digitalWrite(PIN_STEP_X, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_X, 0);
        delayMicroseconds( 500 );
      }
    }
}
