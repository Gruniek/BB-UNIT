String FIRMWARE_VERSION = "0.3.3"; // Software Version
int channel             = 1;       // Remote channel : Put the same of your remote

// FIRMWARE FILE //
#include "Pictures.h"
#include "Setup.h"
#include "Fonctions.h"
#include "Communication.h"
#include "Motor_AB.h"
#include "Motor_X.h"
#include "Motor_Y.h"
#include "Motor_Z.h"
#include "Checker.h"
#include "Text.h"
#include <math.h>

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
 12/2016
 =========
 - Cleaning variable name
 - Add RF433Mhz command for the Head and the controller
 - Add choise the channel

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
  
  // IT IS MANDATORY TO ADD 'c<YOUR CHANNEL>' for use commands. //
  
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
  'STAT \n' // It is all data SENDED to the remote

  < ENABLE >
  'ENABLE a1 x1 z1 \n'    // ENABLE MOTOR AB, XY and Z (a = AB, x=XY, z = Z | 0 = desabled | 1 = enabled )



  FOR AFTER
  http://tutorial.cytron.com.my/2014/05/15/wireless-uart-with-arduino-and-433mhz-or-434mhz-module/
  note : 10, 11, 12, 13, 14, 15, 50, 51, 52, 53

Library used :
- 128x64 OLED screen : https://github.com/adafruit/Adafruit_SSD1306
- MPU6050            : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050




*/


#include "Wire.h"
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>




#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>


//===============//
// YOU CAN TOUCH //
//===============//

// HC-12
bool ifHead = false; // True if the head are wireless connected
SoftwareSerial controler(10, 11); // RX, TX // For the renote communication
SoftwareSerial head(12, 13);   // RX, TX // For the head communication


// RF24
//RF24 radio(54, 55);

// MPU6050

//MPU6050 accelgyroIC1(0x69); // HEAD
//MPU6050 accelgyroIC2(0x69); // BODY

//MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


//int16_t axb, ayb, azb;
//int16_t gxb, gyb, gzb;


// MOTOR CONFIGURATION // YOU CAN TOUCH THIS

bool    INVERT_A       = false; // INVERT THE A MOTOR DIRECTION
bool    INVERT_B       = false; // INVERT THE B MOTOR DIRECTION
bool    INVERT_X       = false; // INVERT THE X MOTOR DIRECTION
bool    INVERT_Y       = true;  // INVERT THE Y MOTOR DIRECTION
bool    INVERT_XY      = false; // INVERT X Y AXIAL

//int     MIN_X          = 45;    // SET THE MIN INCLINAISON FOR X
//int     MAX_X          = 135;   // SET THE MAX INCLINAISON FOR X
int     MIN_Y          = 45;    // SET THE MIN INCLINAISON FOR Y
int     MAX_Y          = 135;   // SET THE MAX INCLINAISON FOR Y

int     ETALON_X       = 0;     // AJUSTEMENT FOR X
int     ETALON_Y       = 0;    // AJUSTEMENT FOR Y
int     ETALON_B_X     = 0;     // AJUSTEMENT FOR X
int     ETALON_B_Y     = -6;    // AJUSTEMENT FOR Y

int     HYSTERESIS     = 1;     // HYSTERESSIS FOR THE ANGLE CALCULATION

//=====================//
// NOW YOU CAN'T TOUCH //
//=====================//

//PINOUT

//#define PIN_ENABLE_A     33      // Activation off all stepper
//#define PIN_ENABLE_B     27      // Activation off all stepper




//#define LIGHT            20      // Pin for turn on/off lighting on the ball (Great for a technical maintenance ;) )



// I2C
int  I2C_ADRESS         = 1;

// MOTOR VARIABLE
int  SPEED_A            = 0;
int  SPEED_B            = 0;
bool RUN_A              = false;
bool RUN_B              = false;
bool DIRECTION_A        = true;   // true = Front | false = back
bool DIRECTION_B        = true;   // true = Front | false = back

int  x                  = 0;
int  y                  = 0;
int  bx                 = 0;
int  by                 = 0;
int  vx                 = 0;
int  vy                 = 0;

//int  SPEED_X            = 0;
int  SPEED_Y            = 0;
int  SPEED_Z            = 0;
int  SETPOINT_X         = 90;
int  SETPOINT_Y         = 90;

//int  Z_ANGLE            = 0;
bool NEW_DATA           = false;

//int  MULTIPLIER         = 15;
int  EMERGENCY_STOP     = 11; // PIN
int  E_STOP             = 12; // PIN
bool EMG_STOP           = false;

int  TMP_DATA[30];

//bool RUN_X              = false;
bool RUN_Y              = false;
bool RUN_Z              = false;
//bool DIRECT_X           = false; // false = Gauche / true = droite
bool DIRECT_Y           = false; // false = Gauche / true = droite
bool DIRECT_Z           = false; // false = Gauche / true = droite
 
bool X_OK               = false;
bool Y_OK               = false;
bool Z_OK               = false;

bool PRODUCTION         = false;
bool LED_PING           = false;
bool BOOT               = false;
bool TEST_SEQUENCE      = false;
bool ENABLE_MOTOR_AB    = false;
bool ENABLE_MOTOR_X     = false;
bool ENABLE_MOTOR_Y     = false;
bool ENABLE_MOTOR_Z     = false;
bool STAT_RUN           = false;

//unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
unsigned long previousMillisZ = 0;
unsigned long previousMillisA = 0;
unsigned long previousMillisB = 0;
unsigned long currentA = 0;
unsigned long currentB = 0;

unsigned long trigGyro = 1000;
unsigned long trigSec  = 1000;
unsigned long trigG    = 0;
unsigned long trigS    = 0;


unsigned char returnBuffer[8],counter=0; //buffer where the received data will be stored
unsigned char fullBuffer=0;              //0: buffer not full, 1: buffer is full
int yawPitchRoll[3];


//=============================================================================//
//   SETUP
//=============================================================================//
void setup()
{
  Serial.begin(115200);

	
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();
  display.clearDisplay();
	
  // PINMODE
  pinMode( PIN_RESET       , OUTPUT );
  
  pinMode( PIN_ENABLE_AB   , OUTPUT );
  pinMode( PIN_ENABLE_X    , OUTPUT );
  pinMode( PIN_ENABLE_Y    , OUTPUT );
  pinMode( PIN_ENABLE_Z    , OUTPUT );
  
  pinMode( PIN_DIRECTION_X , OUTPUT );
  pinMode( PIN_STEP_X      , OUTPUT );
  pinMode( PIN_X_MS_1      , OUTPUT );
  pinMode( PIN_X_MS_2      , OUTPUT );
  pinMode( PIN_X_MS_3      , OUTPUT );
  
  pinMode( PIN_DIRECTION_Y , OUTPUT );
  pinMode( PIN_STEP_Y      , OUTPUT );
  pinMode( PIN_Y_MS_1      , OUTPUT );
  pinMode( PIN_Y_MS_2      , OUTPUT );
  pinMode( PIN_Y_MS_3      , OUTPUT );
  
  pinMode( PIN_ENABLE_Z    , OUTPUT );
  pinMode( PIN_STEP_Z      , OUTPUT );
  pinMode( PIN_DIRECTION_Z , OUTPUT );
  pinMode( PIN_Z_MS_1      , OUTPUT );
  pinMode( Z_POSITION      , OUTPUT );
  
  pinMode( PIN_DIR_A       , OUTPUT );
  pinMode( PIN_STEP_A      , OUTPUT );
  
  pinMode( PIN_DIR_B       , OUTPUT );
  pinMode( PIN_STEP_B      , OUTPUT );  
 
  pinMode( TRIGGER_LIGHT   , INPUT );
  pinMode( LIGHTING        , OUTPUT );
  
  pinMode( LED_1           , OUTPUT );
  pinMode( LED_2           , OUTPUT );
  pinMode( LED_3           , OUTPUT );
  pinMode( LED_4           , OUTPUT );
  pinMode( LED_5           , OUTPUT );
  pinMode( LED_6           , OUTPUT );
  
  pinMode( BUZZER          , OUTPUT );
  pinMode( POWER_1         , OUTPUT );
  pinMode( POWER_2         , OUTPUT );
 
  pinMode( AUX_1           , OUTPUT );
  pinMode( AUX_2           , OUTPUT );
  pinMode( AUX_3           , OUTPUT );
  
  pinMode( BP_LEFT         , INPUT  );
  pinMode( BP_RIGH         , INPUT  );


  
  // SERIAL
  LOGO_TEXT();
  COPY_TEXT();
  
  Serial.print("           FIRMWARE VERSION : ");
  Serial.print(FIRMWARE_VERSION);
  Serial.println("               ");
  Serial.println("==================================================");
  
  Serial.println(" ");
  Serial.println("[1]-Booting up...");
  // Start of the I2C protocol
  Serial.print("[2]-Initializing I2C devices...  I2C ADRESS :");
  Serial.println(I2C_ADRESS);
  Wire.begin(I2C_ADRESS);
  delay(1000);
  
  // Start the GY-25 6050 Gyroscope
  Serial.println("[3]-Connect to the GY-25-6050 HEAD: ");
  Serial1.begin(115200);
  delay(1000);    
  Serial1.write(0XA5);
  Serial1.write(0X52); 
  

  // END OF BOOT/SETUP
  Serial.println("=====================");
  Serial.println("Booting successfull !");
  Serial.println("=====================");
}

//=============================================================================//
//   LOOP
//=============================================================================//
void loop()
{
  unsigned long currentG = millis(); // millis();

  if (currentG - trigG  >= trigGyro && PRODUCTION)
  {
    trigG = currentG;

    // GET MOTION HEAD
    //-------------------------------------------------------------------------//
    
    if(fullBuffer == 1 && returnBuffer[0]==0xAA && returnBuffer[7]==0x55)   //Check if buffer is full and if packet is correct
    {      
        //Convert raw data to angle     
        yawPitchRoll[0]=(returnBuffer[1]<<8|returnBuffer[2])/100;
        yawPitchRoll[1]=(returnBuffer[3]<<8|returnBuffer[4])/100;
        yawPitchRoll[2]=(returnBuffer[5]<<8|returnBuffer[6])/100;

        //Print angles
            
		ax = yawPitchRoll[0];
    	ay = yawPitchRoll[1];
    	ay = yawPitchRoll[2];
    
    	if(INVERT_XY)
    	{
    		x = (ax + ETALON_X + 90);
    		y = (ay + ETALON_Y + 90);
    		
    		//y = (((ax / 180) + ETALON_X) + 90);
      		//x = (((ay / 180) + ETALON_Y) + 90);
      	/*	Serial.print("X=");
      		Serial.print(x);  
      		Serial.print(" Y=");
      		Serial.println(y);
      	*/
    	}
    	else
    	{
    		y = (ax + ETALON_X + 90);
    		x = (ay + ETALON_Y + 90);
    		//x = (((ax / 180) + ETALON_X) + 90);
      		//y = (((ay / 180) + ETALON_Y) + 90);
    	}
    }
     
    fullBuffer=0;
    angleModulePortEvent();         // Checks if data has been received
  }
  
  
  
  char msg[64];
  if (Serial.available() > 0)
  {
    Serial.readBytesUntil('\n', msg, sizeof msg);
    NEW_DATA = true;
  }

  // DATA IN //
  if(NEW_DATA)
  {
    NEW_DATA = false;

    if (strcmp(strtok(msg, " "), "SET") == 0)
    {
      Serial.println("UPDATE SETPOINT");
      // trouvÃ© le message
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'x':
            SETPOINT_X  = val; // SETPOINT_X #1
            break;
          case 'y':
            SETPOINT_Y  = val; // SETPOINT_Y #2
            break;
          case 'i':
            INVERT_XY  = val; // INVERT_XY #4
            break;
          case 'j':
            INVERT_X   = val; // INVERT_X #5
            break;
          case 'k':
            INVERT_Y   = val; // INVERT_Y #6
            break;
          case 'h':
            HYSTERESIS = val; // HYSTERESIS #7
            break;
        }
      }
    }

    if (strcmp(strtok(msg, " "), "RUN") == 0)
    {
   		Serial.println("ROll BB-8 ROLL !");
        PRODUCTION  = true;
    }

    if (strcmp(strtok(msg, " "), "STOP") == 0)
    {
    	Serial.println("STOP");
        PRODUCTION  = false; 
    }
    
    if (strcmp(strtok(msg, " "), "DIE") == 0)
    {
   		software_Reset() ;
    }
    
    if (strcmp(strtok(msg, " "), "HELP") == 0)
    {
   		HELP_TEXT() ;
    }
    
    if (strcmp(strtok(msg, " "), "ABOUT") == 0)
    {
   		COPY_TEXT() ;
    }

    if (strcmp(strtok(msg, " "), "RESET") == 0)
    {
      Serial.println("=   HARD RESET   =");
      delay(1000);
      Serial.println("=       3        =");
      delay(1000);
      Serial.println("=       2        =");
      delay(1000);
      Serial.println("        1        =");
      delay(1000);
      Serial.println("= Wake up Neo... =");
      delay(1000);
      software_Reset() ;
    }
    
    if (strcmp(strtok(msg, " "), "STAT") == 0)
    {
		STAT_RUN = !STAT_RUN;
    }
 
    if (strcmp(strtok(msg, " "), "TEST") == 0)
    {
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'c':
        	Serial.println("");
        	TEST_SEQUENCE  = true;
          break;
        }
      }
    }



    if (strcmp(strtok(msg, " "), "ROTATE") == 0) // ROTATE d1 s100  <-- Rotate Z right, 18c/sec
    {
      // trouvÃ© le message
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'd':
            DIRECT_Z  = val; // DIRECT_Z #8
            break;
          case 's':
            SPEED_Z   = val; // SPEED_Z #9
            break;
        }
      }
    }

    if (strcmp(strtok(msg, " "), "MOVE") == 0)
    {
      Serial.println("UPDATE SETPOINT");
      // trouvÃ© le message
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'a':
            SPEED_A = val; // SPEED_A #10
            break;
          case 'b':
            SPEED_B = val; // SPEED_B #11
            break;
          case 'c':
            DIRECTION_A = val; // DIRECTION_A #12
            break;
          case 'd':
            DIRECTION_B = val; // DIRECTION_B #13
            break;
        }
      }
    }

    if (strcmp(strtok(msg, " "), "ENABLE") == 0)
    {
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'a':
            ENABLE_MOTOR_AB  = val; // ENABLE_MOTOR_AB 
            if(ENABLE_MOTOR_AB)
      		{
      			digitalWrite(ENABLE_MOTOR_AB, 1);
      			Serial.println("Motor A and B Enabled");
      		}
      		else
      		{
      			digitalWrite(ENABLE_MOTOR_AB, 0);
      			Serial.println("Motor A and B Desabled");
      		}
          break;
          
          case 'x':
            ENABLE_MOTOR_X  = val; // ENABLE_MOTOR_XY
            if(ENABLE_MOTOR_X)
    		{
      			digitalWrite(ENABLE_MOTOR_X, 1);
      			Serial.println("Motor X Enabled");
      		}
      		else
      		{
      			digitalWrite(ENABLE_MOTOR_X, 0);
      			Serial.println("Motor X Desabled");
      		}
          break;
          
          case 'Y':
            ENABLE_MOTOR_X  = val; // ENABLE_MOTOR_XY 
            if(ENABLE_MOTOR_Y)
      		{
      			digitalWrite(ENABLE_MOTOR_Y, 1);
      			Serial.println("Motor Y Enabled");
      		}
      		else
      		{
      			digitalWrite(ENABLE_MOTOR_Y, 0);
      			Serial.println("Motor Y Desabled");
      		}
          break;
          
          case 'z':
            ENABLE_MOTOR_Z   = val; // ENABLE_MOTOR_Z 
            if(ENABLE_MOTOR_Z)
      		{
      			digitalWrite(ENABLE_MOTOR_Z, 1);
      			Serial.println("Motor Z Enabled");
      		}
      		else
      		{
      			digitalWrite(ENABLE_MOTOR_Z, 0);
      			Serial.println("Motor Z Desabled");
      		}
          break;
        }
      }
    }
    
  }

//  digitalWrite( PIN_ENABLE_A, ENABLE_MOTOR_AB );
//  digitalWrite( PIN_ENABLE_B, ENABLE_MOTOR_XY );


  if(BOOT)
  {
  	
  }
  
  
  // PRODUCTION CODE
  //-------------------------------------------------------------------------//
  if(PRODUCTION)  //- PRODUCTION && !EMG_STOP
  {
    //===========================
    //        d8888 888888b.   //
    //       d88888 888  "88b  //
    //      d88P888 888  .88P  //
    //     d88P 888 8888888K.  //
    //    d88P  888 888  "Y88b //
    //   d88P   888 888    888 //
    //  d8888888888 888   d88P //
    // d88P     888 8888888P"  //
    //===========================
    MOTOR_AB();
    //===============
    // Y88b   d88P //
    //  Y88b d88P  //
    //   Y88o88P   //
    //    Y888P    //
    //    d888b    //
    //   d88888b   //
    //  d88P Y88b  //
    // d88P   Y88b //
    //===============
	MOTOR_X(x, SETPOINT_X);
    //===============
    // Y88b   d88P //
    //  Y88b d88P  //
    //   Y88o88P   //
    //    Y888P    //
    //     888     //
    //     888     //
    //     888     //
    //     888     //
    //===============
    MOTOR_Y(y, SETPOINT_Y);
    //===============
    // 8888888888P // 
    //       d88P  //
    //      d88P   //
    //     d88P    //
    //    d88P     // 
    //   d88P      //
    //  d88P       //
    // d8888888888 //
    //===============
    MOTOR_Z();
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
/*
    // IF THE MPU6050 ARE ALWAYS CONNECTED, IF NOT, HE RECONNECT AUTOMATICLY
    if(!accelgyro.testConnection());
    //if(!accelgyroIC1.testConnection());
    {
      PRODUCTION = false;
      EMG_STOP = true;
      accelgyro.initialize();
      //accelgyroIC1.initialize();
      //if(accelgyro.testConnection())
      if(accelgyro.testConnection()) PRODUCTION = true;
    }
*/
    

    // SEND STATUT
    // ON SERIAL USB
    
    
    if(STAT_RUN)
    {
    	Serial.print("STAT > ");
    	Serial.print("X = ");
    	Serial.print(x);
    	Serial.print(" / y = ");
    	Serial.print(y);
    	Serial.print(" / SETPOINT X = ");
    	Serial.print(SETPOINT_X);
    	Serial.print(" / SETPOINT Y = ");
    	Serial.print(SETPOINT_Y);
		
		Serial.println(" ");
    }
  } // ENF OF PRODUCTION
} // END OF LOOP



void angleModulePortEvent()
{
  while (Serial1.available())
  {   
    returnBuffer[counter]=(unsigned char)Serial1.read();
    if(counter==0&&returnBuffer[0]!=0xAA) return;            //wait for start character
    counter++;       
    if(counter==8)                
    {    
       counter=0;                                           
       fullBuffer=1;                                        //mark buffer as full
    }      
  }
} 
