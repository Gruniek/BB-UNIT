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

#define pinEnable 8 // Activation du driver/pilote

#define pinStep    2 //2 Signal de PAS (avancement)
#define pinDir     5 //5 Direction 



#define pinStepY    4 // 4Signal de PAS (avancement)
#define pinDirY     7 // 7Direction 


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// MPU6050
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// I2C
int adress = 2;
int LED    = 13; // PIN

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

int motorX         = 2;  // PIN 5
int enableX        = 8;  // PIN7
int dirX           = 5;  // PIN6
int motorY         = 4;  // PIN 8
int enableY        = 9; // PIN10
int dirY           = 7;  // PIN9
int motorZ         = 3;  // PIN3
int dirZ           = 10;  // PIN2
int enableZ        = 4;  // PIN4

int multiplier     = 15;
int emergencyStop  = 11; // PIN
int eStop          = 12; // PIN

bool runX          = false; 
bool runY          = false; 
bool directX       = false; // false = Gauche / true = droite
bool directY       = false; // false = Gauche / true = droite

// PID
unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
long                    intX  = 0;
long                    intY  = 0; 

bool start = false;


void setup()
{
  pinMode( pinEnable, OUTPUT );
  pinMode( pinDir   , OUTPUT );
  pinMode( pinStep  , OUTPUT );
  pinMode( pinDirY   , OUTPUT );
  pinMode( pinStepY  , OUTPUT );
  
    Serial.begin(9600);
    
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
          case 'x': setpointX = val; break;
          case 'y': setpointY = val; break;
      	  case 'm': multiplier = val; break;
        }
      }
    }
    
    if (strcmp(strtok(msg, " "), "RUN") == 0)
    {
      Serial.println("RUN");
      start = true;
    }
    
    if (strcmp(strtok(msg, " "), "STOP") == 0)
    {
      Serial.println("STOP");
      start = false;
    }
  }
   
//  Serial.print(x); Serial.print("/"); Serial.println(y);
  
  if(start)  //-
  {

  // X //
  
  // Direction
  if(x < 90) directX = false;
  else directX = true;

  digitalWrite(pinDir, directX); // dirX
  
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
    speedX = speedX / multiplier; // DEL
  //if(speedX < 15) speedX = 15;
  

  // Mise en route du moteur
  unsigned long currentX = millis();

  
    if (currentX - previousMillisX >= speedX) 
    {
      previousMillisX = currentX;
      
      if(runX)
      {
        digitalWrite(pinStep, 1);
        delayMicroseconds( 500 );
        digitalWrite(pinStep, 0);
        delayMicroseconds( 500 );
        /*Serial.print(">X< : STEP ");
        Serial.print(directX);
        Serial.print(" ");
        Serial.println(x);*/
      }
  }

  // Y //
  
  // Direction
  if(y < 90) directY = false;
  else directY = true;

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
  speedY = speedY / multiplier; // DEL
  //if(speedY < 15) speedY = 15;
  

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
       /* Serial.print("<Y> : STEP ");
        Serial.print(directY);
        Serial.print(" ");
        Serial.println(y);*/
      }
  } 
  
  } //-
}
