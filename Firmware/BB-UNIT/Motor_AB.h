#include <Arduino.h>

void MOTOR_AB()
{
	/*
	//======================================================================
    //        d8888
    //       d88888
    //      d88P888
    //     d88P 888
    //    d88P  888
    //   d88P   888
    //  d8888888888
    // d88P     888
    //======================================================================
    // MOTOR A (Left)

    // Run the motor
    if(SPEED_A > 0) RUN_A = true;
    else RUN_A = false;
    SPEED_A = 1000 / SPEED_A;

    // Motor direction / With inversion
    if(INVERT_A)
    {
      if(DIRECTION_A) DIRECTION_A = false;
      else DIRECTION_A = true;
    }
    digitalWrite(PIN_DIR_A, DIRECTION_A); // Set the direction

    // Send Step inpultion to the motor
    unsigned long currentA = millis(); // millis();
    if (currentA - previousMillisA >= SPEED_A)
    {
      previousMillisA = currentA;

      if(RUN_A)
      {
        digitalWrite(PIN_STEP_A, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_A, 0);
        delayMicroseconds( 500 );
      }
    }

    //======================================================================
    // 888888b.
    // 888  "88b
    // 888  .88P
    // 8888888K.
    // 888  "Y88b
    // 888    888
    // 888   d88P
    // 8888888P"
    //======================================================================
    // MOTOR B (Right)

    // Run the motor
    if(SPEED_B > 0) RUN_B = true;
    else RUN_B = false;
    SPEED_B = 1000 / SPEED_B;

    // Motor direction / With inversion
    if(INVERT_B)
    {
      if(DIRECTION_B) DIRECTION_B = false;
      else DIRECTION_B = true;
    }
    digitalWrite(PIN_DIR_B, DIRECTION_B); // Set the direction

    // Send Step inpultion to the motor
    unsigned long currentB = millis(); // millis();
    if (currentB - previousMillisB >= SPEED_B)
    {
      previousMillisB = currentB;

      if(RUN_B)
      {
        digitalWrite(PIN_STEP_B, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_B, 0);
        delayMicroseconds( 500 );
      }
    }
*/
}
