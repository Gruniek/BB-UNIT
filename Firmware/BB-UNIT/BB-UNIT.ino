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
  'BOOT c1\n'  // LAUNCH TEST for X, Y and Z -> Return X and Y to initial SETPOINT (x90 y90 ) and Z to ZERO

  < RUN >
  'RUN c1\n'   // RUN the code in production mode (Recieved all data, X and Y runing for the SETPOINT poistion and Z folow the remote controll

  < STOP >
  'STOP c1\n'  // STOP the production mode. All motors are stopped.

  < ROTATE >
  'ROTATE d1 s100 c1\n'   // ROTATE Z motor on the RIGHT DIRECTION (d0 LEFT, d1 RIGHT) WITH the SPEED at 1000/100 = 10 step/sec = 18dec/sec

  < STAT >
  'STAT c1\n' // It is all data SENDED to the remote

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
MPU6050 accelgyroIC1(0x68); // HEAD
MPU6050 accelgyroIC2(0x69); // BODY

//MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


int16_t axb, ayb, azb;
int16_t gxb, gyb, gzb;


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
int     ETALON_Y       = -6;    // AJUSTEMENT FOR Y
int     ETALON_B_X     = 0;     // AJUSTEMENT FOR X
int     ETALON_B_Y     = -6;    // AJUSTEMENT FOR Y

int     HYSTERESIS     = 1;     // HYSTERESSIS FOR THE ANGLE CALCULATION

//=====================//
// NOW YOU CAN'T TOUCH //
//=====================//

//PINOUT

#define PIN_ENABLE_A     33      // Activation off all stepper
#define PIN_ENABLE_B     27      // Activation off all stepper

#define PIN_STEP_A       31      // PIN STEP FOR X
#define PIN_DIR_A        29      // PIN DIRECTION FOR X 

#define PIN_STEP_B       25      // PIN STEP FOR X
#define PIN_DIR_B        23      // PIN DIRECTION FOR X 

#define pinStepX         26      // PIN STEP FOR X
#define pinDirX          24      // PIN DIRECTION FOR X 

#define pinStepY         4       // PIN STEP FOR Y
#define pinDirY          3       // PIN DIRECTION FOR Y

#define pinStepZ         8       // PIN STEP FOR Y
#define pinDirZ          9       // PIN DIRECTION FOR Y
#define Z_POSITION       35      // PIN FOR THE DIGITAL INPUT FOR THE HEAD POSITION


#define TRIGGER_LIGHT    36      // PIN TRIGGER FOR START/STOP THE LIGHTING INSIDE THE BALL
#define LIGHTING         37      // PIN FOR FEED THE LIGHTING

#define SWITCH_1         34      // TRIGGER NOT USED FOR THE MOMENT

#define LED_1            48      // LED 1
#define LED_2            44      // LED 2
#define LED_3            40      // LED 3
#define LED_4            46      // LED 4
#define LED_5            42      // LED 5


#define PING             13

#define LIGHT            20      // Pin for turn on/off lighting on the ball (Great for a technical maintenance ;) )

#define CURRENT          0
#define VOLTAGE          6

// I2C
int I2C_ADRESS         = 1;

// MOTOR VARIABLE
int SPEED_A            = 0;
int SPEED_B            = 0;
bool RUN_A             = false;
bool RUN_B             = false;
bool DIRECTION_A       = true;   // true = Front | false = back
bool DIRECTION_B       = true;   // true = Front | false = back

int x                  = 0;
int y                  = 0;
int bx                 = 0;
int by                 = 0;
int vx                 = 0;
int vy                 = 0;

int speedX             = 0;
int speedY             = 0;
int speedZ             = 0;
int setpointX          = 90;
int setpointY          = 90;

int Zangle             = 0;
bool newData           = false;

int multiplier         = 15;
int emergencyStop      = 11; // PIN
int eStop              = 12; // PIN
bool emgStop           = false;

int TMP_DATA[30]      ;

bool runX              = false;
bool runY              = false;
bool runZ              = false;
bool directX           = false; // false = Gauche / true = droite
bool directY           = false; // false = Gauche / true = droite
bool directZ           = false; // false = Gauche / true = droite

bool xok               = false;
bool yok               = false;
bool zok               = false;

long intX              = 0;
long intY              = 0;

bool production        = false;
bool LED_PING          = false;
bool BOOT              = false;

bool enMotAB           = false;
bool enMotXY           = false;
bool enMotZ            = false;



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
	pinMode( PIN_ENABLE_A  , OUTPUT );
	pinMode( PIN_ENABLE_B  , OUTPUT );
	pinMode( pinDirX       , OUTPUT );
	pinMode( pinStepX      , OUTPUT );
	pinMode( pinDirY       , OUTPUT );
	pinMode( pinStepY      , OUTPUT );
	pinMode( PIN_DIR_A     , OUTPUT );
	pinMode( PIN_STEP_A    , OUTPUT );
	pinMode( PIN_DIR_B     , OUTPUT );
	pinMode( PIN_STEP_B    , OUTPUT );
	pinMode( LIGHTING      , OUTPUT );
	pinMode( LED_1         , OUTPUT );
	pinMode( LED_2         , OUTPUT );
	pinMode( LED_3         , OUTPUT );
	pinMode( LED_4         , OUTPUT );
	pinMode( LED_5         , OUTPUT );
	  
	pinMode( Z_POSITION    , INPUT  );
	pinMode( TRIGGER_LIGHT , INPUT  );
	pinMode( SWITCH_1      , INPUT  );
	
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

	// Start the MPU6050 Gyroscope
	Serial.print("[3]-Connect to the MPU6050 HEAD: ");
	//accelgyro.initialize();
	accelgyroIC1.initialize();
	Serial.println(accelgyroIC1.testConnection() ? "Connection successful" : "Connection failed");

	Serial.print("[4]-Connect to the MPU6050 BODY: ");
    accelgyroIC2.initialize();
   	Serial.println(accelgyroIC2.testConnection() ? "Connection successful" : "Connection failed");

	//Serial.println(accelgyro.testConnection());
	//Serial.println(accelgyro.testConnection() ? "Connection successful" : "Connection failed");

	// Start the RF24 Radio controll
	//Serial.println("[4]-Initializing RF24 devices...  ");
	//radio.begin();
	//radio.openReadingPipe(0, 00001);
	//radio.startListening();

	// Start remote connexion
	Serial.println("[5]-Initializing HC-12 devices [REMOTE] ");
	controler.begin(9600);

	// Start remote connexion
	if(ifHead)
	{
		Serial.println("[6]-Initializing HC-12 devices [ HEAD ] ");
		head.begin(9600);
	}
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

	if (currentG - trigG  >= trigGyro)
	{
		trigG = currentG;

		// GET MOTION HEAD
		//-------------------------------------------------------------------------//
		//accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		accelgyroIC1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if(INVERT_XY)
		{
			y = (((ax / 180) + ETALON_X) + 90);
			x = (((ay / 180) + ETALON_Y) + 90);
		}
		else
		{
			x = (((ax / 180) + ETALON_X) + 90);
			y = (((ay / 180) + ETALON_Y) + 90);
		}
		
		
		// GET MOTION BODY
		//-------------------------------------------------------------------------//
		//accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		accelgyroIC2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if(INVERT_XY)
		{
			by = (((axb / 180) + ETALON_B_X) + 90);
			bx = (((ayb / 180) + ETALON_B_Y) + 90);
		}
		else
		{
			bx = (((axb / 180) + ETALON_B_X) + 90);
			by = (((ayb / 180) + ETALON_B_Y) + 90);
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

	if (controler.available()) // IF RF24 ---> radio.available()
	{
		char msg[64] = {0};
		controler.readBytesUntil('\n', msg, sizeof msg); // IF RF24 ---> radio.read(&msg, sizeof(msg));
		newData = true;
	}

	if (head.available()) 
	{
		char msg[64] = {0};
		controler.readBytesUntil('\n', msg, sizeof msg); // IF RF24 ---> radio.read(&msg, sizeof(msg));
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
					case 'x':
						TMP_DATA[1]  = val; // setpointX #1
						break;
					case 'y':
						TMP_DATA[2]  = val; // setpointY #2
						break;
					case 'm':
						TMP_DATA[3] = val; // multiplier #3
						break;
					case 'i':
						TMP_DATA[4]  = val; // INVERT_XY #4
						break;
					case 'j':
						TMP_DATA[5]   = val; // INVERT_X #5
						break;
					case 'k':
						TMP_DATA[6]   = val; // INVERT_Y #6
						break;
					case 'h':
						TMP_DATA[7] = val; // HYSTERESIS #7
						break;
					case 'c':
						TMP_DATA[0] = val;  
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				setpointX  = TMP_DATA[1];
				setpointY  = TMP_DATA[2];
				multiplier = TMP_DATA[3];
				INVERT_XY  = TMP_DATA[4];
				INVERT_X   = TMP_DATA[5];
				INVERT_Y   = TMP_DATA[6];
				HYSTERESIS = TMP_DATA[7];
			}
			
		}

		if (strcmp(strtok(msg, " "), "RUN") == 0)
		{
			char *p;
			while ((p = strtok(NULL, " ")) != NULL)
			{
				int val = atoi(p + 1);
				switch (*p)
				{
					case 'c':
						TMP_DATA[0]  = val; 
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				Serial.println("ROll BB-8 ROLL !");
				production = true;
				TMP_DATA[0] = 0;
			}
		}

		if (strcmp(strtok(msg, " "), "STOP") == 0)
		{
			char *p;
			while ((p = strtok(NULL, " ")) != NULL)
			{
				int val = atoi(p + 1);
				switch (*p)
				{
					case 'c':
						TMP_DATA[0]  = val; 
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				Serial.println("STOP");
				production = false;
				TMP_DATA[0] = 0;
			}

		}

		if (strcmp(strtok(msg, " "), "RESET") == 0)
		{
			char *p;
			while ((p = strtok(NULL, " ")) != NULL)
			{
				int val = atoi(p + 1);
				switch (*p)
				{
					case 'c':
						TMP_DATA[0]  = val; 
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				Serial.println("RESET");
				production = false;
				emgStop    = false;
				TMP_DATA[0] = 0;
			}	

		}

		if (strcmp(strtok(msg, " "), "BOOT") == 0)
		{
			char *p;
			while ((p = strtok(NULL, " ")) != NULL)
			{
				int val = atoi(p + 1);
				switch (*p)
				{
					case 'c':
						TMP_DATA[0]  = val; 
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				BOOT = true;
				Serial.println("BOOTING UP...");
				TMP_DATA[0] = 0;
				delay(2000);
			}	

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
					case 'd':
						TMP_DATA[8]  = val; // directZ #8
						break;
					case 's':
						TMP_DATA[9]   = val; // speedZ #9
						break;
					case 'c':
						TMP_DATA[0]   = val; // channel #0
						break;
				}
			}
			
			if(TMP_DATA[0] == channel )
			{
				directZ = TMP_DATA[8];
				speedZ  = TMP_DATA[9];
				TMP_DATA[0] = 0;
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
					case 'a':
						TMP_DATA[10] = val; // SPEED_A #10
						break;
					case 'b':
						TMP_DATA[11] = val; // SPEED_B #11
						break;
					case 'e':
						TMP_DATA[12] = val; // DIRECTION_A #12
						break;
					case 'd':
						TMP_DATA[13] = val; // DIRECTION_B #13
						break;
					case 'c':
						TMP_DATA[0]   = val; // channel #0
						break;
				}
			}

			if(TMP_DATA[0] == channel )
			{
				SPEED_A     = TMP_DATA[10];
				SPEED_B     = TMP_DATA[11];
				DIRECTION_A = TMP_DATA[12];
				DIRECTION_B = TMP_DATA[13];
				TMP_DATA[0] = 0;
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
					case 'a':
						TMP_DATA[14]  = val; // enMotAB #14
						break;
					case 'x':
						TMP_DATA[15]  = val; // enMotXY #15
						break;
					case 'z':
						TMP_DATA[16]   = val; // enMotZ #16
						break;
					case 'c':
						TMP_DATA[0]   = val; // channel #0
						break;
				}
			}
		
			if(TMP_DATA[0] == channel )
			{
				enMotAB     = TMP_DATA[14];
				enMotXY     = TMP_DATA[15];
				enMotZ      = TMP_DATA[16];
				TMP_DATA[0] = 0;
			}
		}
	}

	digitalWrite( PIN_ENABLE_A, enMotAB );
	digitalWrite( PIN_ENABLE_B, enMotXY );


	if(BOOT)
	{
		enMotAB = true;
		enMotXY = true;
		enMotZ  = true;

		// SET X IN POSITION
		if(x < 90) directX = false;
		else directX = true;

		if(INVERT_X)
		{
			if(directX) directX = false;
			else directX = true;
		}
		digitalWrite(pinDirX, directX); // dirX

		if(x > (setpointX + HYSTERESIS || (x < setpointX - HYSTERESIS || !xok )))
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

		if(INVERT_Y)
		{
			if(directY) directY = false;
			else directY = true;
		}
		digitalWrite(pinDirY, directY); // dirX

		if(y > (setpointY + HYSTERESIS) || (y < setpointY - HYSTERESIS || !yok))
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

		if(INVERT_X)
		{
			if(directX) directX = false;
			else directX = true;
		}
		digitalWrite(pinDirX, directX); // dirX

		// Si ont doit demarer le moteur
		if(x > (setpointX + HYSTERESIS) || (x < setpointX - HYSTERESIS ))
		{
			if(x > MIN_X && x < MAX_X) runX = true;
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
		if(INVERT_Y)
		{
			if(directY) directY = false;
			else directY = true;
		}
		digitalWrite(pinDirY, directY);

		// Si ont doit demarer le moteur
		if(y > (setpointY + HYSTERESIS) || (y < setpointY - HYSTERESIS ))
		{
			if(y > MIN_Y && y < MAX_Y) runY = true;
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
		//if(!accelgyro.testConnection());
		if(!accelgyroIC1.testConnection());
		{
			production = false;
			emgStop = true;
			//accelgyro.initialize();
			accelgyroIC1.initialize();
			//if(accelgyro.testConnection())
			if(accelgyroIC1.testConnection())
			{
				production = true;
			}
		}

		

		// SEND STATUT
		// ON SERIAL USB
		Serial.print("STAT x");
		Serial.print(x);
		Serial.print(" y");
		Serial.print(y);
		Serial.print(" i");
		Serial.print(INVERT_XY);
		Serial.print(" j");
		Serial.print(INVERT_X);
		Serial.print(" k");
		Serial.print(INVERT_XY);
		Serial.print(" m");
		Serial.print(multiplier);
		Serial.print(" z");
		Serial.print(Zangle);

		Serial.println(" ");

		// SEND STATUT
		// ON CONTROLER RF433
				// ON SERIAL USB
		controler.print("STAT x");
		controler.print(x);
		controler.print(" y");
		controler.print(y);
		controler.print(" i");
		controler.print(INVERT_XY);
		controler.print(" j");
		controler.print(INVERT_X);
		controler.print(" k");
		controler.print(INVERT_Y);
		controler.print(" m");
		controler.print(multiplier);
		controler.print(" z");
		controler.print(Zangle);
		
		
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
