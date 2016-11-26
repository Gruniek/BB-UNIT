String version = "0.3.1"; // Software Version
int channel    = 1;       // Remote channel : Put the same of your remote
/*
  - OPEN BB-X -
 A open source BB-8 for create you own BB-8!
 ###########################################

 https://github.com/Gruniek/OPEN-BB-X
 Made by Daniel M/Gruniek/
 
 Compatible only with an Arduino MEGA 2560 and the specific PCB.
 PCB Link : https://github.com/Gruniek/OPEN-BB-X/tree/master/Head/Stabiliser/PCB
 
 --------------
 - Change log -
 --------------
 
 11/2016
 =========
 - Conversion for a Arduino Mega 2560 Board
 - ADD motor A and B
 - FIX alls bug for stabilisation
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
  // It is all data SENDED to the remote
  
  < ENABLE > 
  'ENABLE a1 x1 z1 \n'    // ENABLE MOTOR AB, XY and Z (a = AB, x=XY, z = Z | 0 = desabled | 1 = enabled )
  
  
  
  FOR AFTER 
  http://tutorial.cytron.com.my/2014/05/15/wireless-uart-with-arduino-and-433mhz-or-434mhz-module/
  note : 10, 11, 12, 13, 14, 15, 50, 51, 52, 53
  
*/

#include "Wire.h"
#include "MPU6050.h"
#include "SoftwareSerial.h"
//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>


//===================//
// NOW YOU CAN TOUCH //
//===================//

// HC-12
bool ifHead = false; // True if the head are wireless connected
SoftwareSerial remote(10, 11); // RX, TX // For the renote communication
SoftwareSerial head(12, 13);   // RX, TX // For the head communication


// RF24
//RF24 radio(54, 55);

// MPU6050
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// MOTOR CONFIGURATION // YOU CAN TOUCH THIS 

bool    invertA        = false; // INVERT THE A MOTOR DIRECTION
bool    invertB        = false; // INVERT THE B MOTOR DIRECTION
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

// 
//=====================//
// NOW YOU CAN'T TOUCH //
//=====================//

//PINOUT

#define pinEnableXY      2      // Activation off all stepper
#define pinEnableAB      3
#define pinEnableZ       4

#define pinStepA         5      // PIN STEP FOR X
#define pinDirA          6      // PIN DIRECTION FOR X 

#define pinStepB         7      // PIN STEP FOR X
#define pinDirB          8      // PIN DIRECTION FOR X 

#define pinStepX         9      // PIN STEP FOR X
#define pinDirX          10      // PIN DIRECTION FOR X 

#define pinStepY         11      // PIN STEP FOR Y
#define pinDirY          12      // PIN DIRECTION FOR Y

#define pinStepZ         14      // PIN STEP FOR Y
#define pinDirZ          15      // PIN DIRECTION FOR Y

#define Z_POSITION       16      // PIN FOR THE DIGITAL INPUT FOR THE HEAD POSITION

#define PING             13 

#define CurrentBat1      0
#define CurrentBat2      1
#define CurrentBat3      2
#define CurrentBat4      3
#define Pinvoltage       6

// I2C
int adress         = 2;

// MOTOR VARIABLE
int speedA         = 0;
int speedB         = 0;
bool runA          = false;
bool runB          = false;
bool dirA          = true;   // true = Front | false = back
bool dirB          = true;   // true = Front | false = back

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
bool newData       = false;

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

long intX          = 0;
long intY          = 0; 

bool production    = false;
bool LED_PING      = false;
bool BOOT          = false;

bool enMotAB       = false;
bool enMotXY       = false;
bool enMotZ        = false;

unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
unsigned long previousMillisZ = 0;
unsigned long previousMillisA = 0;
unsigned long previousMillisB = 0;
unsigned long currentA = 0;
unsigned long currentB = 0;

unsigned long trigGyro = 100;
unsigned long trigSec  = 1000;
unsigned long trigG    = 0;
unsigned long trigS    = 0;


//=============================================================================//
//   SETUP
//=============================================================================//
void setup()
{
    // PINMODE
    pinMode( pinEnableAB , OUTPUT );
    pinMode( pinEnableXY , OUTPUT );
    pinMode( pinEnableZ  , OUTPUT );
    pinMode( pinDirX     , OUTPUT );
    pinMode( pinStepX    , OUTPUT );
    pinMode( pinDirY     , OUTPUT );
    pinMode( pinStepY    , OUTPUT );
    pinMode( pinDirA     , OUTPUT );
    pinMode( pinStepA    , OUTPUT );
    pinMode( pinDirB     , OUTPUT );
    pinMode( pinStepB    , OUTPUT );
    pinMode( PING        , OUTPUT );
    
    pinMode( Z_POSITION  , INPUT  );
    
    // SERIAL
    Serial.begin(9600);

    // Blabla, blablabla
    Serial.println("==================================================");
    Serial.println("  _______ ______     _     _  ______  ____ ______ "); 
    Serial.println(" (____   (____  )   | |   | |/ ___  |(____|______)");
    Serial.println("  ____)  )____)  )__| |   | | |   | | | |   | |   ");      
    Serial.println(" |  __  (|  __  (___) |   | | |   | | | |   | |   ");
    Serial.println(" | |__)  ) |__)  )  | |___| | |   | |_| |_  | |   ");
    Serial.println(" |______/|______/   |______/|_|   |_(_____) |_|   ");
    Serial.println("            _ Astromech Industrie _               ");
    Serial.println("      > https://github.com/Gruniek/BB-UNIT <      ");
    Serial.println("           > http://r2builders.fr/ <              ");
    Serial.print("           FIRMWARE VERSION : ");
    Serial.print(version);
    Serial.println("               ");
    Serial.println("==================================================");
    
    Serial.println(" ");
    Serial.println("[1]-Booting up...");
   
    // Start of the I2C protocol
    Serial.print("[2]-Initializing I2C devices...  I2C ADRESS :");
    Serial.println(adress);
    Wire.begin(adress);
    
    // Start the MPU6050 Gyroscope
    Serial.print("[3]-Connect to the MPU6050 : ");
    accelgyro.initialize();
    //Serial.println(accelgyro.testConnection());
    Serial.println(accelgyro.testConnection() ? "Connection successful" : "Connection failed");
    
    // Start the RF24 Radio controll
    //Serial.println("[4]-Initializing RF24 devices...  ");
    //radio.begin();
    //radio.openReadingPipe(0, 00001);
    //radio.startListening();
 
    // Start remote connexion
    Serial.println("[4]-Initializing HC-12 devices [REMOTE] ");
    remote.begin(9600);

    // Start remote connexion
    if(ifHead)
    {
	Serial.println("[5]-Initializing HC-12 devices [ HEAD ] ");
        remote.begin(9600);
    }
    // END OF BOOT/SETUP
    Serial.println("====================");    
    Serial.println("Booting successful !");
    Serial.println("====================");
}

//=============================================================================//
//   LOOP
//=============================================================================//
void loop()
{
    unsigned long currentG = millis(); // millis();

    if (currentG - trigG  >= trigGyro) 
    {
    	trigG = currentG;
      
         // GET MOTION
         //-------------------------------------------------------------------------// 
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
  
    // GET SERIAL AND RF24 DATA
    //-------------------------------------------------------------------------//
	  
    char msg[64];
    if (Serial.available() > 0)
    {
    	Serial.readBytesUntil('\n', msg, sizeof msg);
    	newData = true;
    }
	
    if (remote.available()) // IF RF24 ---> radio.available()
    {
    	char msg[64] = {0};
    	radio.readBytesUntil('\n', msg, sizeof msg);
    	newData = true;
    }
	
    if(newData)
    {
	Serial.println(msg);
	newData = false;
		
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
    	
    	if (strcmp(strtok(msg, " "), "RESET") == 0)
    	{
    	    Serial.println("RESET");
    	    production = false;
    	    emgStop    = false;
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
    	
    	if (strcmp(strtok(msg, " "), "MOVE") == 0)
    	{
            Serial.println("UPDATE SETPOINT");
      	    // trouvé le message
      	    char *p;
      	    while ((p = strtok(NULL, " ")) != NULL)
      	    {
                int val = atoi(p + 1);
        	switch (*p)
        	{
          	     case 'a': speedA  = val; break;
          	     case 'b': speedB  = val; break;
      	  	     case 'c': dirA    = val; break;
      		     case 'd': dirB    = val; break;
        	}
      	    }
        }
    	
    	if (strcmp(strtok(msg, " "), "ENABLE") == 0)
    	{
       	    //Serial.println("UPDATE SETPOINT");
  	    // trouvé le message
  	    char *p;
      	    while ((p = strtok(NULL, " ")) != NULL)
   	    {
      	        int val = atoi(p + 1);
        	switch (*p)
        	{
        	    case 'a': enMotAB  = val; break;
        	    case 'x': enMotXY  = val; break;
      		    case 'z': enMotZ   = val; break;
        	}
      	    }
    	}
    	
    }
   
   digitalWrite(pinEnableAB, enMotAB);
   digitalWrite(pinEnableXY, enMotXY);
   digitalWrite(pinEnableZ, enMotZ);
   
   if(BOOT)
   {
 	enMotAB = true;
 	enMotXY = true;
 	enMotZ  = true;
 	
   	// SET X IN POSITION
   	if(x < 90) directX = false;
  	else directX = true;
  
  	if(invertX) 
  	{
    		if(directX) directX = false;
    		else directX = true;
  	}
  	digitalWrite(pinDirX, directX); // dirX
  		
  	if(x > (setpointX + hysteresis) || (x < setpointX - hysteresis || !xok ))
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
  		
  	if(y > (setpointY + hysteresis) || (y < setpointY - hysteresis || !yok))
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
  	if(!digitalRead(Z_POSITION))
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
    	enMotAB = true;
 	enMotXY = true;
 	enMotZ  = true;
    	
    	//======================================================================
    	//        d8888 
    	//       d88888 
     	//	d88P888 
    	//     d88P 888 
   	//    d88P  888 
  	//   d88P   888 
 	//  d8888888888 
	// d88P     888
	//======================================================================
    	// MOTOR A (Left)
    	
	// Run the motor
	if(speedA > 0) runA = true;
	else runA = false;
	speedA = 1000 / speedA;
		
	// Motor direction / With inversion
	if(invertA) 
  	{
    	    if(dirA) dirA = false;
	    else dirA = true;
	}
	digitalWrite(pinDirA, dirA); // Set the direction
		
	// Send Step inpultion to the motor
	unsigned long currentA = millis(); // millis();
	if (currentA - previousMillisA >= speedA) 
	{
	    previousMillisA = currentA;
	    
	    if(runA)
	    {
	        digitalWrite(pinStepA, 1);
	        delayMicroseconds( 500 );
	        digitalWrite(pinStepA, 0);
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
	if(speedB > 0) runB = true;
	else runB = false;
	speedB = 1000 / speedB;
	
	// Motor direction / With inversion
	if(invertB) 
 	{
	    if(dirB) dirB = false;
	    else dirB = true;
	}
	digitalWrite(pinDirB, dirB); // Set the direction
		
	// Send Step inpultion to the motor
	unsigned long currentB = millis(); // millis();
	if (currentB - previousMillisB >= speedB) 
	{
	    previousMillisB = currentB;
	      
	    if(runB)
	    {
	        digitalWrite(pinStepB, 1);
	        delayMicroseconds( 500 );
	        digitalWrite(pinStepB, 0);
	        delayMicroseconds( 500 );
	     }
	} 

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
  	if(x < setpointX) vx = (setpointX - x); 
  	if(x == 0) vx = 0;
  	if(x > setpointX) vx = (x - setpointX);
  
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
  	if(y < setpointY) vy = (setpointY - y); 
  	if(y == 0) vy = 0;
  	if(y > setpointY) vy = (y - setpointY);
  
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
  
    	//======================================================================
	// 8888888888P 
        //       d88P  
        //      d88P   
    	//     d88P    
        //    d88P     
        //   d88P      
        //  d88P        
        // d8888888888 
	//======================================================================  		
		
	digitalWrite(pinDirZ, directZ);
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
	    
    }

    // TRIGGER / SEC
    //-------------------------------------------------------------------------// 
    unsigned long currentS = millis(); // millis();

    if (currentS - trigS  >= trigSec) 
    {
    	trigS = currentS;
    	
    	// PING
    	if(LED_PING) LED_PING = false;
    	else LED_PING = true;
    	digitalWrite(PING, LED_PING);
    	
	// IF THE MPU6050 ARE ALWAYS CONNECTED, IF NOT, HE RECONNECT AUTOMATICLY
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
    } // ENF OF PRODUCTION
} // END OF LOOP

// VOID FOR TAKE THE CURRENT
int current(int Cpin)
{
    int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
    int RawValue= 0;
    int ACSoffset = 2500; 
    double Voltage = 0;
    double Amps = 0;

    RawValue = analogRead(Cpin);
    Voltage = (RawValue / 1024.0) * 5000; // Gets you mV
    Amps = ((Voltage - ACSoffset) / mVperAmp);
 
    return(Amps);
}

// VOID FOR TAKE THE BATTERY VOLTAGE
int voltage(int Vpin)
{
    float vout = 0.0;
    float vin = 0.0;
    float R1 = 100000.0; // resistance of R1 (100K) -see text!
    float R2 = 10000.0; // resistance of R2 (10K) - see text!
    int value = 0;

    value = analogRead(Vpin);
    vout = (value * 5.0) / 1024.0;
    vin = vout / (R2/(R1+R2)); 
    if (vin<0.09) vin=0.0;
   
    return(vin);
}
