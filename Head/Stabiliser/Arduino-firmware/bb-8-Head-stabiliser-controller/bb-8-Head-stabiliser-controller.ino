/*
  - OPEN BB-X -
 A open source BB-8 for create you own BB-8!
 ###########################################

 https://github.com/Gruniek/OPEN-BB-X
 Made by Daniel M/Gruniek/
 
 Compatible only with an Arduino Nano and the specific PCB.
 PCB Link : https://github.com/Gruniek/OPEN-BB-X/tree/master/Head/Stabiliser/PCB
 
 --------------
 - Change log -
 --------------
 
 10/1016
 ==========
 - Add PID for X and Y
 
 
 To do list
 ==========
 - Add Z rotation controller
 - Add Rx/Tx from the motherboard for change de coordinate of the head, 
  rotate the head and report to the remote the status off all sensors/positiom.
  
  
  Big thanks for http://r2builders.fr/ !
  Initial project : http://r2builders.fr/forum/viewtopic.php?f=26&t=3928&hilit=BB8+par+MOUS

*/

#include "Wire.h"
#include "MPU6050.h"



// MOTOR CONFIGURATION // YOU CAN TOUCH THIS 

#define pinEnable        4      // Activation off all stepper

#define pinStepX         3      // PIN STEP FOR X
#define pinDirX          2      // PIN DIRECTION FOR X 

#define pinStepY         6      // PIN STEP FOR Y
#define pinDirY          5      // PIN DIRECTION FOR Y

#define pinStepY         8      // PIN STEP FOR Y
#define pinDirY          7      // PIN DIRECTION FOR Y

#define Z_POSITION       9      // PIN FOR THE DIGITAL INPUT FOR THE HEAD POSITION

bool    invertX        = false; // INVERT THE X MOTOR DIRECTION
bool    invertY        = true;  // INVERT THE Y MOTOR DIRECTION
bool    invertXY       = false; // INVERT X Y AXIAL 

int     minX           = 45;    // SET THE MIN INCLINAISON FOR X
int     maxX           = 135;   // SET THE MAX INCLINAISON FOR X
int     minY           = 45;    // SET THE MIN INCLINAISON FOR Y
int     maxY           = 135;   // SET THE MAX INCLINAISON FOR Y

int     etalonX        = 0;     // AJUSTEMENT FOR X
int     etalonY        = -6;    // AJUSTEMENT FOR Y

int     hysteresis     = 1;     // HYSTERESSIS FOR THE ANGLE CALCULATION


//=====================//
// NOW YOU CAN'T TOUCH //
//=====================//

// MPU6050
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// I2C
int adress = 2;
int PING   = 13; // PIN

// VARIABLE
int x              = 0; 
int y              = 0; 
int vx             = 0; 
int vy             = 0; 

int speedX         = 0;
int speedY         = 0;

int setpointX      = 90; 
int setpointY      = 90; 



int multiplier     = 15;
int emergencyStop  = 11; // PIN
int eStop          = 12; // PIN

bool runX          = false; 
bool runY          = false; 
bool directX       = false; // false = Gauche / true = droite
bool directY       = false; // false = Gauche / true = droite

bool xok           = false;
bool yok           = false;
bool zok           = false;

// PID
unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
long                    intX  = 0;
long                    intY  = 0; 


unsigned long trigGyro = 100;
unsigned long trigSec  = 1000;
unsigned long trigG    = 0;
unsigned long trigS    = 0;


bool production = false;
bool LED_PING   = false;
bool BOOT = false;




//=============================================================================//
//   SETUP
//=============================================================================//
void setup()
{
	pinMode( pinEnable , OUTPUT );
    pinMode( pinDirX   , OUTPUT );
    pinMode( pinStepX  , OUTPUT );
    pinMode( pinDirY   , OUTPUT );
    pinMode( pinStepY  , OUTPUT );
    pinMode( PING      , OUTPUT );
    pinMode( Z_POSITION, INPUT  );
    
    
    Serial.begin(9600);
    
    Serial.println("Initializing I2C devices...");
    Serial.print("I2C Adress = ");
    Serial.println(adress);
  
    Wire.begin(adress);
    //Wire.onReceive(); // register event

    
    accelgyro.initialize();


    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

//=============================================================================//
//   LOOP
//=============================================================================//
void loop()
{
    // GET MOTION
    //-------------------------------------------------------------------------//
    unsigned long currentG = millis(); // millis();

    if (currentG - trigG  >= trigGyro) 
    {
    	trigG = currentG;
      
      
    	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if(invertXY)
		{
			y = (((ax/180)+etalonX)+90);
  			x = (((ay/180)+etalonY)+90);
		}
		else
		{
			x = (((ax/180)+etalonX)+90);
  			y = (((ay/180)+etalonY)+90);
		}
    }
  
    // GET SERIAL DATA
    //-------------------------------------------------------------------------//
	  
    char msg[30];
    if (Serial.available() > 0)
    {
    	Serial.readBytesUntil('\n', msg, sizeof msg);

    	if (strcmp(strtok(msg, " "), "SET") == 0)
    	{
        	Serial.println("UPDATE SETPOINT");
      		// trouvé le message
      		char *p;
      		while ((p = strtok(NULL, " ")) != NULL)
      		{
        		int val = atoi(p + 1);
        		switch (*p)
        		{
          			case 'x': setpointX  = val; break;
          			case 'y': setpointY  = val; break;
      	  			case 'm': multiplier = val; break;
      				case 'i': invertXY   = val; break;
      				case 'j': invertX    = val; break;
      				case 'k': invertX    = val; break;
      				case 'h': hysteresis = val; break;
        		}
      		}
    	}
    
    	if (strcmp(strtok(msg, " "), "RUN") == 0)
    	{
        	Serial.println("ROll BB-8 ROLL !");
      	    production = true;
    	}
    
    	if (strcmp(strtok(msg, " "), "STOP") == 0)
    	{
    		Serial.println("STOP");
    	    production = false;
    	}
    	
    	if (strcmp(strtok(msg, " "), "BOOT") == 0)
    	{
    		Serial.println("STOP");
    	    BOOT = true;
    	    Serial.println("BOOTING UP...");
    	    delay(2000);
    	}
    	
    	
  }
   
   if(boot)
   {
   	    // SET X IN POSITION
   		if(x < 90) directX = false;
  		else directX = true;
  
  		if(invertX) 
  		{
    		if(directX) directX = false;
    		else directX = true;
  		}
  		digitalWrite(pinDirX, directX); // dirX
  		
  		if(x > (setpointX + hysteresis) || (x < setpointX - hysteresis ))
  		{
  	        digitalWrite(pinStepX, 1);
        	delayMicroseconds( 500 );
    	    digitalWrite(pinStepX, 0);
    	    delayMicroseconds( 500 );
  		}
  		else
  		{
  			if(!xok) 
  			{
  				Serial.println(STAT x1);
  				xok = true;
  				delay(2000);
  			}
  		}
  		
  		
  		// SET Y IN POSITION
   		if(y < 90) directY = false;
  		else directY = true;
  
  		if(invertY) 
  		{
    		if(directY) directY = false;
    		else directY = true;
  		}
  		digitalWrite(pinDirY, directY); // dirX
  		
  		if(x > (setpointY + hysteresis) || (y < setpointY - hysteresis ))
  		{
  	        digitalWrite(pinStepY, 1);
        	delayMicroseconds( 500 );
    	    digitalWrite(pinStepY, 0);
    	    delayMicroseconds( 500 );
  		}
  		else
  		{
  			if(!yok) 
  			{
  				Serial.println(STAT y1);
  				yok = true;
  				delay(2000);
  			}
  		}
  		
  		// SET W IN POSITION
  		
  		if()
  
  
   }
   
	// PRODUCTION CODE
    //-------------------------------------------------------------------------// 
    if(production)  //-
    { 

  // X //
  
  // Direction
  if(x < 90) directX = false;
  else directX = true;
  
  if(invertX) 
  {
    if(directX) directX = false;
    else directX = true;
  }
  
  digitalWrite(pinDirX, directX); // dirX
  
  // Si ont doit demarer le moteur
  if(x > (setpointX + hysteresis) || (x < setpointX - hysteresis )) 
  { 
    if(x > minX && x < maxX) runX = true; 
    else runX = false;
  }
  else runX = false;
  
  
  // Calcul du PID X
  if(x < 90) vx = (90 - x); 
  if(x == 0) vx = 0;
  if(x > 90) vx = (x - 90);
  
  speedX = 1000 / (vx);
  speedX = speedX * multiplier; // DEL
  if(speedX < 15) speedX = 15;
  

  // Mise en route du moteur
  unsigned long currentX = millis();

  
    if (currentX - previousMillisX >= speedX) 
    {
      previousMillisX = currentX;
      
      if(runX)
      {
        digitalWrite(pinStepX, 1);
        delayMicroseconds( 500 );
        digitalWrite(pinStepX, 0);
        delayMicroseconds( 500 );
      }
  }

  // Y //
  
  // Direction
  if(y < 90) directY = false;
  else directY = true;
  if(invertY) 
  {
    if(directY) directY = false;
    else directY = true;
  }
  digitalWrite(pinDirY, directY);
  
  // Si ont doit demarer le moteur
  if(y > (setpointY + hysteresis) || (y < setpointY - hysteresis )) 
  { 
    if(y > minY && y < maxY) runY = true; 
    else runY = false;
  }
  else runY = false;
  
  // Calcul du PID X
  if(y < 90) vy = (90 - y); 
  if(y == 0) vy = 0;
  if(y > 90) vy = (y - 90);
  
  speedY = 1000 / (vy);
  speedY = speedY * multiplier; // DEL
  if(speedY < 15) speedY = 15;
  

  // Mise en route du moteur
  unsigned long currentY = millis(); // millis();

    if (currentY - previousMillisY >= speedY) 
    {
      previousMillisY = currentY;
      
      if(runY)
      {
        digitalWrite(pinStepY, 1);
        delayMicroseconds( 500 );
        digitalWrite(pinStepY, 0);
        delayMicroseconds( 500 );
      }
  } 
  
  } //-
  
  
  	// SEND SERIAL DATA
    //-------------------------------------------------------------------------// 
    unsigned long currentS = millis(); // millis();

    if (currentS - trigS  >= trigSec) 
    {
    	trigS = currentS;
    	
    	
    	
    	// PING
    	if(LED_PING) LED_PING = false;
    	else LED_PING = true;
    	digitalWrite(PING, LED_PING);
    	
    	
    	// SEND STATUT
    	Serial.print("SEND x");
    	Serial.print(x);
    	Serial.print(" y");
    	Serial.print(y);
    	Serial.print(" i");
    	Serial.print(invertXY);
    	Serial.print(" j");
    	Serial.print(invertX);
    	Serial.print(" k");
    	Serial.print(invertY);
    	Serial.print(" m");
    	Serial.print(multiplier);
    	Serial.println(" ");
    }
}
