String version = "0.1.2";
/*

 >      FIRMWARE ONLY FOR AN ARDUINO NANO !!    <
 > IF YOU USE ANOTHER ARDUINO, ADAPT THE PINOUT <
 
 ###################################################
  ______  ______     _     _ ______  _____ _______ 
 (____  \(____  \   | |   | |  ___ \(_____|_______)
  ____)  )____)  )__| |   | | |   | | | | | |         
 |  __  (|  __  (___) |   | | |   | | | | | |      
 | |__)  ) |__)  )  | |___| | |   | |_| |_| |_____ 
 |______/|______/    \______|_|   |_(_____)\______)
    A open source project for create you own BB-8!
 ###################################################

 https://github.com/Gruniek/BB-UNIT
 Made by Daniel M/Gruniek/MOUS
 
 Compatible only with an Arduino Nano and the specific PCB.
 PCB Link : https://github.com/Gruniek/BB-UNIT/tree/master/Head/Stabiliser/PCB
 
 --------------
 - Change log -
 --------------
 
 11/2016
 =========
 - FIX alls bug foe stabilisation
 - ADD BOOT, STAT, ROTATE command
 - ADD Z motor configuration and production
 - ADD BOOT mode, for initilize or reinizialize the Droid position
 
 10/2016
 ==========
 - Add PID for X and Y
 
 
 To do list
 ==========
 - Add Z rotation controller
 - Add Rx/Tx from the motherboard for change de coordinate of the head, 
  rotate the head and report to the remote the status off all sensors/positiom.
  
  
  Big thanks for http://r2builders.fr/ !
  Initial project : http://r2builders.fr/forum/viewtopic.php?f=26&t=3928&hilit=BB8+par+MOUS
  
  ===============
  EXEMPLE COMMAND 
  ===============
  < SET >
  'SET x87 y75 h3 \n'        // SET a new setpoint for x and y with 3degree of Hysteressis
  'SET x90 y90 i1 j0 k1 \n'  // SET an inversion of the sensor X Y and invert the direction of X
  
  	x = SETPOINT for X
	y = SETPOINT for Y
	m = MULTIPLIER FOR THE SPEED OF THE MOTOR X and Y
	i = INVERT the angle sensor ( X = Y and Y = X )
	j = INVERT the direction of X
	k = INVERT the direction of Y
	h = SET a new HYSTERESSIS 



  < BOOT > 
  'BOOT \n'  // LAUNCH TEST for X, Y and Z -> Return X and Y to initial SETPOINT (x90 y90 ) and Z to ZERO



  < RUN >
  'RUN \n'   // RUN the code in production mode (Recieved all data, X and Y runing for the SETPOINT poistion and Z folow the remote controll
  
  < STOP >   
  'STOP \n'  // STOP the production mode. All motors are stopped.
  
  
  < ROTATE >
  'ROTATE d1 s100 \n'   // ROTATE Z motor on the RIGHT DIRECTION (d0 LEFT, d1 RIGHT) WITH the SPEED at 1000/100 = 10 step/sec = 18dec/sec
  
  
  < STAT >
  // It is all data SENDED to the MASTER-BOARD
  
*/

#include "Wire.h"
#include "MPU6050.h"



// MOTOR CONFIGURATION // YOU CAN TOUCH THIS 

#define pinEnable        4      // Activation off all stepper

#define pinStepX         3      // PIN STEP FOR X
#define pinDirX          2      // PIN DIRECTION FOR X 

#define pinStepY         6      // PIN STEP FOR Y
#define pinDirY          5      // PIN DIRECTION FOR Y

#define pinStepZ         8      // PIN STEP FOR Y
#define pinDirZ          7      // PIN DIRECTION FOR Y

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
int adress         = 2;
int PING           = 13; // PIN
int PING_MASTER    = 12;
// VARIABLE
int x              = 0; 
int y              = 0; 
int vx             = 0; 
int vy             = 0; 

int speedX         = 0;
int speedY         = 0;
int speedZ         = 0;
int setpointX      = 90; 
int setpointY      = 90; 

int Zangle         = 0;


int multiplier     = 15;
int emergencyStop  = 11; // PIN
int eStop          = 12; // PIN
bool emgStop       = false;

bool runX          = false; 
bool runY          = false; 
bool runZ          = false;
bool directX       = false; // false = Gauche / true = droite
bool directY       = false; // false = Gauche / true = droite
bool directZ       = false; // false = Gauche / true = droite

bool xok           = false;
bool yok           = false;
bool zok           = false;

// PID
unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
unsigned long previousMillisZ = 0;

long                    intX  = 0;
long                    intY  = 0; 


unsigned long trigGyro = 100;
unsigned long trigSec  = 1000;
unsigned long trigG    = 0;
unsigned long trigS    = 0;


bool production = false;
bool LED_PING   = false;
bool BOOT = false;

bool ifPing    = false;
int tmpPing    = 0;
int tmpPing2   = 0;
int pingMaster = 0;

//=============================================================================//
//   SETUP
//=============================================================================//
void setup()
{
    pinMode( pinEnable   , OUTPUT );
    pinMode( pinDirX     , OUTPUT );
    pinMode( pinStepX    , OUTPUT );
    pinMode( pinDirY     , OUTPUT );
    pinMode( pinStepY    , OUTPUT );
    pinMode( PING        , OUTPUT );
    pinMode( PING_MASTER , INPUT  );
    pinMode( Z_POSITION  , INPUT  );
    
    
    Serial.begin(9600);
  
    Serial.println(" ");
    Serial.println("#######################################");
    Serial.println("#         Astromech Industrie         #");
    Serial.println("#                BB-UNIT              #");
    Serial.println("# https://github.com/Gruniek/BB-UNIT/ #");
    Serial.println("#######################################");
    Serial.println(" ");
    Serial.print("  BB-8 Version ");
    Serial.println(version);
    Serial.println("===================");
    Serial.println(" ");
      
    Serial.print("Initializing I2C devices...  I2C ADRESS :");
    Serial.print(adress);  
    Wire.begin(adress);
      
    accelgyro.initialize();
    Serial.print("Connect to the MPU6050 : ");
    
    Serial.println(accelgyro.testConnection() ? "Connection successful" : "Connection failed");
    
    Serial.println("Booting successful !");
    Serial.println("====================");
    

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
      	    emgStop = false;
    	}
    
    	if (strcmp(strtok(msg, " "), "STOP") == 0)
    	{
    		Serial.println("STOP");
    	    production = false;
    	}
    	
    	if (strcmp(strtok(msg, " "), "RESET") == 0)
    	{
    		Serial.println("RESET");
    	    production = false;
    	}
    	
    	if (strcmp(strtok(msg, " "), "BOOT") == 0)
    	{
    		
    	    BOOT = true;
    	    Serial.println("BOOTING UP...");
    	    delay(2000);
    	}
    	
    	if (strcmp(strtok(msg, " "), "ROTATE") == 0) // ROTATE d1 s100  <-- Rotate Z right, 18c/sec
    	{
      		// trouvé le message
      		char *p;
      		while ((p = strtok(NULL, " ")) != NULL)
      		{
        		int val = atoi(p + 1);
        		switch (*p)
        		{
          			case 'd': directZ  = val; break;
          			case 's': speedZ   = val; break;
        		}
      		}
    	}
    	
    	
  }
   
   if(BOOT)
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
  				Serial.println("BOOT x1 ");
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
  				Serial.println("BOOT y1 ");
  				yok = true;
  				delay(2000);
  			}
  		}
  		
  		// SET Z IN POSITION
  		
  		if(!Z_POSITION)
  		{
  		  	digitalWrite(pinStepZ, 1);
        	delayMicroseconds( 500 );
    	    digitalWrite(pinStepZ, 0);
    	    delayMicroseconds( 500 );	
  		}
  		else
  		{
  		  	if(!zok) 
  			{
  				Serial.println("BOOT z1 ");
  				zok = true;
  				Zangle = 0;
  				delay(2000);
  			}	
  		}
  		
  		if( xok && yok && zok)
  		{
			Serial.println("STAT b1 ");
  			Serial.println("Booting UP OK!");
  			Serial.println("SEND 'RUN \n' for start the production");
  			BOOT = false;
  		}
  
  
   }
   
	// PRODUCTION CODE
    //-------------------------------------------------------------------------// 
    if(production && !emgStop)  //-
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
  
  		// Calcul du PID Y
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
  
  		//--- Z
  		if(speedZ != 0)
  		{
  			unsigned long currentZ = millis();
  			if (currentZ - previousMillisZ >= speedZ) 
    		{
      			previousMillisZ = currentZ;
  
    	 		digitalWrite(pinStepY, 1);
    	    	delayMicroseconds( 500 );
    	    	digitalWrite(pinStepY, 0);
    	    	delayMicroseconds( 500 );
    	    	if(directZ) Zangle++;
    	    	else Zangle--;
  			} 
  		}
  		if(Zangle == 201) Zangle = 0;
  		if(Zangle == -1)  Zangle = 0;
  		
  
  } //-
 
  // Check if the Master-Board are ALIVE
 
  if(digitalRead(PING_MASTER) && !ifPing)
  {
  	ifPing = true;
  	pingMaster++;
  }
  
  if(digitalRead(PING_MASTER) && ifPing) ifPing = false;
  
  
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
    	
    	
    	tmpPing2++;
    	
    	if(pingMaster > tmpPing) 
    	{
    		tmpPing = pingMaster;
    		tmpPing2 = 0;
    	}
    	if(tmpPing2 > 3) emgStop = true;
    	
    	
    	
    	if(!accelgyro.testConnection());
    	{
    		production = false;
    		emgStop = true;
    		accelgyro.initialize();
    		if(accelgyro.testConnection())
    		{
    			production = true;
    		}
    	}
    	
    	
    	
    	/*
    	
    	// SEND STATUT
    	Serial.print("STAT x");
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
    	Serial.print(" z");
    	Serial.print(Zangle);
    	
    	Serial.println(" ");
    	
    	*/
    }
}
