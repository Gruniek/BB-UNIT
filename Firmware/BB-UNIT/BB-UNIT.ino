String FIRMWARE_VERSION = "0.3.3"; // Software Version
int channel             = 1;       // Remote channel : Put the same of your remote


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
//#include "MPU6050.h"
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

int     MIN_X          = 45;    // SET THE MIN INCLINAISON FOR X
int     MAX_X          = 135;   // SET THE MAX INCLINAISON FOR X
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

#define PIN_STEP_A       4      // PIN STEP FOR X
#define PIN_DIR_A        5      // PIN DIRECTION FOR X 
#define PIN_STEP_B       6      // PIN STEP FOR X
#define PIN_DIR_B        7      // PIN DIRECTION FOR X 
#define PIN_ENABLE_AB    9 

#define PIN_ENABLE_X     43
#define PIN_STEP_X       51      // PIN STEP FOR X        //26
#define PIN_DIRECTION_X  53      // PIN DIRECTION FOR X   //24 
#define PIN_X_MS_1       45
#define PIN_X_MS_2       47
#define PIN_X_MS_3       49

#define PIN_ENABLE_Y     31     
#define PIN_STEP_Y       39       // PIN STEP FOR Y       // 4
#define PIN_DIRECTION_Y  41       // PIN DIRECTION FOR Y  // 3
#define PIN_Y_MS_1       33
#define PIN_Y_MS_2       35
#define PIN_Y_MS_3       37

#define PIN_ENABLE_Z     23
#define PIN_STEP_Z       27      // PIN STEP FOR Y
#define PIN_DIRECTION_Z  29       // PIN DIRECTION FOR Y
#define PIN_Z_MS_1       25
#define Z_POSITION       22

#define BP_LEFT          52
#define BP_RIGH          50

#define LED_1            48
#define LED_2            46
#define LED_3            44
#define LED_4            42
#define LED_5            40
#define LED_6            38

#define TRIGGER_LIGHT    24      // PIN TRIGGER FOR START/STOP THE LIGHTING INSIDE THE BALL
#define LIGHTING         26      // PIN FOR FEED THE LIGHTING

#define POWER_1          28
#define POWER_2          30
#define AUX_1			 32
#define AUX_2            34
#define AUX_3            36

#define BUZZER           3

#define PING             13
#define PIN_RESET        12


//#define LIGHT            20      // Pin for turn on/off lighting on the ball (Great for a technical maintenance ;) )

//#define CURRENT          0
//#define VOLTAGE          6

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

int  SPEED_X            = 0;
int  SPEED_Y            = 0;
int  SPEED_Z            = 0;
int  SETPOINT_X         = 90;
int  SETPOINT_Y         = 90;

//int  Z_ANGLE            = 0;
bool NEW_DATA           = false;

int  MULTIPLIER         = 15;
int  EMERGENCY_STOP     = 11; // PIN
int  E_STOP             = 12; // PIN
bool EMG_STOP           = false;

int  TMP_DATA[30];

bool RUN_X              = false;
bool RUN_Y              = false;
bool RUN_               = false;
bool DIRECT_X           = false; // false = Gauche / true = droite
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

unsigned long previousMillisX = 0;
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

  // Blabla, blablabla
  Serial.println("==================================================");
  Serial.println("  _______ ______     _     _  ______  ____ ______ ");
  Serial.println(" (____   (____  )   | |   | |/ ___  |(____|______)");
  Serial.println("  ____)  )____)  )__| |   | | |   | | | |   | |   ");
  Serial.println(" |  __  (|  __  (___) |   | | |   | | | |   | |   ");
  Serial.println(" | |__)  ) |__)  )  | |___| | |   | |_| |_  | |   ");
  Serial.println(" |______/|______/   |______/|_|   |_(_____) |_|   ");
  Serial.println("==================================================");
  Serial.println("                                                  ");
  Serial.println("            _ Astromech Industrie _               ");
  Serial.println("      > https://github.com/Gruniek/BB-UNIT <      ");
  Serial.println("           > http://r2builders.fr/ <              ");
  Serial.println("                                                  ");
  Serial.println("           > Code by Daniel Mostosi <             ");
  Serial.println("                                                  ");
  Serial.println("                Report an issues?                 ");
  Serial.println("    https://github.com/Gruniek/BB-UNIT/issues     ");
  Serial.println("                                                  ");
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
          case 'm':
            MULTIPLIER = val; // MULTIPLIER #3
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

    if (strcmp(strtok(msg, " "), "RESET") == 0)
    {
      Serial.println("=      HARD RESET       =");
      delay(1000);
      Serial.println("=           3           =");
      delay(1000);
      Serial.println("=           2           =");
      delay(1000);
      Serial.println("=           1           =");
      delay(1000);
      Serial.println("= Going to sleep Neo... =");
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


  /*if(BOOT)
  {
//    ENABLE_MOTOR_AB = true;
    ENABLE_MOTOR_XY = true;
//    ENABLE_MOTOR_Z  = true;
digitalWrite(PIN_ENABLE_XY, ENABLE_MOTOR_XY);
    // SET X IN POSITION
    if(x < 90) DIRECT_X = false;
    else DIRECT_X = true;

    if(INVERT_X)
    {
      if(DIRECT_X) DIRECT_X = false;
      else DIRECT_X = true;
    }
    digitalWrite(PIN_DIRECTION_X, DIRECT_X); // dirX

    if(x > (SETPOINT_X + HYSTERESIS || (x < SETPOINT_X - HYSTERESIS || !X_OK )))
    {
      digitalWrite(PIN_STEP_X, 1);
      delayMicroseconds( 500 );
      digitalWrite(PIN_STEP_X, 0);
      delayMicroseconds( 500 );
    }
    else
    {
      if(!X_OK)
      {
        Serial.println("BOOT x1 ");
        X_OK = true;
        delay(2000);
      }
    }

    // SET Y IN POSITION
    if(y < 90) DIRECT_Y = false;
    else DIRECT_Y = true;

    if(INVERT_Y)
    {
      if(DIRECT_Y) DIRECT_Y = false;
      else DIRECT_Y = true;
    }
    digitalWrite(PIN_DIRECTION_Y, DIRECT_Y); // dirX

    if(y > (SETPOINT_Y + HYSTERESIS) || (y < SETPOINT_Y - HYSTERESIS || !Y_OK))
    {
      digitalWrite(PIN_STEP_Y, 1);
      delayMicroseconds( 500 );
      digitalWrite(PIN_STEP_Y, 0);
      delayMicroseconds( 500 );
    }
    else
    {
      if(!Y_OK)
      {
        Serial.println("BOOT y1 ");
        Y_OK = true;
        delay(2000);
      }
    }
/*
    // SET Z IN POSITION
    if(!digitalRead(Z_POSITION))
    {
      digitalWrite(PIN_STEP_Z, 1);
      delayMicroseconds( 500 );
      digitalWrite(PIN_STEP_Z, 0);
      delayMicroseconds( 500 );
    }
    else
    {
      if(!Z_OK)
      {
        Serial.println("BOOT z1 ");
        Z_OK = true;
        Z_ANGLE = 0;
        delay(2000);
      }
    }
*/
 /*   if( X_OK && Y_OK)
    {
      Serial.println("STAT b1 ");
      Serial.println("Booting UP OK!");
      Serial.println("SEND 'RUN \n' for start the PRODUCTION");
      BOOT = false;
    }
  }
*/
  // PRODUCTION CODE
  //-------------------------------------------------------------------------//
  if(PRODUCTION)  //- PRODUCTION && !EMG_STOP
  {

/*    ENABLE_MOTOR_AB = true;*/
 //   ENABLE_MOTOR_XY = true;
//    ENABLE_MOTOR_Z  = true;


/*

    //======================================================================
    //        d8888
    //       d88888
    //  d88P888
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


//digitalWrite(PIN_ENABLE_XY, ENABLE_MOTOR_XY);
    // Direction
    if(x < 90) DIRECT_X = false;
    else DIRECT_X = true;

    if(INVERT_X)
    {
      if(DIRECT_X) DIRECT_X = false;
      else DIRECT_X = true;
    }
    digitalWrite(PIN_DIRECTION_X, DIRECT_X); // dirX

    // Si ont doit demarer le moteur
    if(x > (SETPOINT_X + HYSTERESIS) || (x < SETPOINT_X - HYSTERESIS ))
    {
      
      if(x > MIN_X && x < MAX_X) RUN_X = true;
      else RUN_X = false;
    }
    else RUN_X = false;

    // Calcul du PID X
    if(x < SETPOINT_X) vx = (SETPOINT_X - x);
    if(x == 0) vx = 0;
    if(x > SETPOINT_X) vx = (x - SETPOINT_X);

    SPEED_X = 1000 / (vx);
    SPEED_X = SPEED_X * MULTIPLIER; // DEL
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
    if(y < 90) DIRECT_Y = false;
    else DIRECT_Y = true;
    if(INVERT_Y)
    {
      if(DIRECT_Y) DIRECT_Y = false;
      else DIRECT_Y = true;
    }
    digitalWrite(PIN_DIRECTION_Y, DIRECT_Y);

    // Si ont doit demarer le moteur
    if(y > (SETPOINT_Y + HYSTERESIS) || (y < SETPOINT_Y - HYSTERESIS ))
    {
      if(y > MIN_Y && y < MAX_Y) RUN_Y = true;
      else RUN_Y = false;
    }
    else RUN_Y = false;

    // Calcul du PID Y
    if(y < SETPOINT_Y) vy = (SETPOINT_Y - y);
    if(y == 0) vy = 0;
    if(y > SETPOINT_Y) vy = (y - SETPOINT_Y);

    SPEED_Y = 1000 / (vy);
    SPEED_Y = SPEED_Y * MULTIPLIER; // DEL
    if(SPEED_Y < 15) SPEED_Y = 15;

    // Mise en route du moteur
    unsigned long currentY = millis(); // millis();

    if (currentY - previousMillisY >= SPEED_Y)
    {
      previousMillisY = currentY;

      if(RUN_Y)
      {
        digitalWrite(PIN_STEP_Y, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_Y, 0);
        delayMicroseconds( 500 );
      }
    }

/*
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

    digitalWrite(PIN_DIRECTION_Z, DIRECT_Z);
    if(SPEED_Z != 0)
    {
      unsigned long currentZ = millis();

      if (currentZ - previousMillisZ >= SPEED_Z)
      {
        previousMillisZ = currentZ;

        digitalWrite(PIN_STEP_Y, 1);
        delayMicroseconds( 500 );
        digitalWrite(PIN_STEP_Y, 0);
        delayMicroseconds( 500 );
        if(DIRECT_Z) Z_ANGLE++;
        else Z_ANGLE--;
      }
    }

    if(Z_ANGLE == 201) Z_ANGLE = 0;
    if(Z_ANGLE == -1)  Z_ANGLE = 0;
*/
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

// VOID FOR TAKE THE CURRENT
int current(int Cpin)
{
  int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
  int RawValue = 0;
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
  vin = vout / (R2 / (R1 + R2));
  if (vin < 0.09) vin = 0.0;

  return(vin);
}



// LOGO
/*
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
  */
  
  
  static const unsigned char PROGMEM logo_16_[] =
{ 
B00000001, B10000000,
B00000010, B01000000,
B00000100, B00011000,
B00001000, B00000010,
B00010000, B00000001,
B01000000, B00000001,
B00100000, B00011000,
B00010000, B00000000,
B00000100, B01000000,
B00000001, B10000000,
 };
 
 
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

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
	asm volatile ("  jmp 0");  
}  
