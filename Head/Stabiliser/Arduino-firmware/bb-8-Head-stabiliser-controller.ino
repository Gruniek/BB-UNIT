/*
  - OPEN BB-X -
 A open source BB-8 for create you own BB-8!
 ###########################################

 https://github.com/Gruniek/OPEN-BB-X
 Made by Daniel M/Gruniek/MOUS
 
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
 - Add Rx/Tx with i2c from the motherboard for change de coordinate of the head, 
  rotate the head and report to the remote the status off all sensors/positiom.
  
  
  Big thanks for http://r2builders.fr/ !
  Initial project : http://r2builders.fr/forum/viewtopic.php?f=26&t=3928&hilit=BB8+par+MOUS

*/



#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// MPU6050
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// I2C
int adress = 2;
int LED    = 13;

// VARIABLE
int x              = 0; 
int y              = 0; 
int vx             = 0; 
int vy             = 0; 
int minX           = 45;
int maxX           = 135;
int minY           = 45;
int maxY           = 135;
int speedX         = 0;
int speedY         = 0;
int etalonX        = 0;  
int etalonY        = -6;
int setpointX      = 90; 
int setpointY      = 90; 
int hysteresis     = 1; 
int motorX         = 2; 
int dirX           = 3; 
int motorY         = 4; 
int dirY           = 5;  
bool runX          = false; 
bool runY          = false; 
bool directX       = false; // false = Gauche / true = droite
bool directY       = false; // false = Gauche / true = droite

// PID
unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
long                    intX  = 0;
long                    intY  = 0; 




void setup()
{

    Serial.begin(115200);
    
	Serial.println("Initializing I2C devices...");
	Serial.print("I2C Adress = ");
	Serial.println(adress);
	
	Wire.begin(adress);
    //Wire.onReceive(); // register event

    
    accelgyro.initialize();


    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED, OUTPUT);
	pinMode(motorX, OUTPUT);
	pinMode(motorY, OUTPUT);
	pinMode(dirX, OUTPUT);
	pinMode(dirY, OUTPUT);

}

void loop()
{
	// GET MOTION
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	x = (((ax/180)+etalonX)+90);
	y = (((ay/180)+etalonY)+90);
	
//	Serial.print(x); Serial.print("/"); Serial.println(y);
	
	// X //
	
	// Direction
	if(x < 90) directX = false;
	else directX = true;

	digitalWrite(dirX, directX);
	
	// Si ont doit demarer le moteur
	if(x > (setpointX + hysteresis) || (x < setpointX - hysteresis )) 
	{ 
		if(x > minX && x < maxX) runX = true; 
		else runY = false;
	}
	else runX = false;
	
	
	// Calcul du PID X
	if(x < 90) vx = (90 - x);	
	if(x == 0) vx = 0;
	if(x > 90) vx = (x - 90);
	
	speedX = 1000 / (vx);
	if(speedX < 15) speedX = 15;
	

	// Mise en route du moteur
	unsigned long currentX = millis();

	
    if (currentX - previousMillisX >= speedX) 
    {
    	previousMillisX = currentX;
    	
    	if(runX)
    	{
    		digitalWrite(motorX, 0);
    		delay(15);
    		digitalWrite(motorX, 1);
    		Serial.print(">X< : STEP ");
    		Serial.print(directX);
    		Serial.print(" ");
    		Serial.println(x);
    	}
	}

	// Y //
	
	// Direction
	if(y < 90) directY = false;
	else directY = true;

	digitalWrite(dirY, directY);
	
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
	if(speedY < 15) speedY = 15;
	

	// Mise en route du moteur
	unsigned long currentY = millis();

    if (currentY - previousMillisY >= speedY) 
    {
    	previousMillisY = currentY;
    	
    	if(runY)
    	{
    		digitalWrite(motorY, 0);
    		delay(15);
    		digitalWrite(motorY, 1);
    		Serial.print("<Y> : STEP ");
    		Serial.print(directY);
    		Serial.print(" ");
    		Serial.println(y);
    	}
	}	
}
