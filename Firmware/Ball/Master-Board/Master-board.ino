String version = "0.1.1";
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
 PCB Link : https://github.com/Gruniek/BB-UNIT/
 
 --------------
 - Change log -
 --------------
 
 11/2016
 =========
 - Start of the code 
 
 To do list
 ==========
 - All
  
  
  Big thanks for http://r2builders.fr/ !
  Initial project : http://r2builders.fr/forum/viewtopic.php?f=26&t=3928&hilit=BB8+par+MOUS
  
  ===============
  EXEMPLE COMMAND 
  ===============
  
*/

#include "Wire.h"
#include "MPU6050.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

int i2cAdress      = 1;

int pinDirA        = 2;      // PIN DIRECTION FOR X 
int pinStepA       = 3;      // PIN STEP FOR X
int pinEnable      = 4;      // Activation off all stepper

int pinDirB        = 5;      // PIN DIRECTION FOR Y
int pinStepB       = 6;      // PIN STEP FOR Y



int PING           = 13; 
int PING_MASTER    = 12;

// MPU6050
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


// RF24
int msg[1];
RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int lastmsg = 1;
String theMessage = "";

void setup()
{
    pinMode( pinEnable   , OUTPUT );
    pinMode( pinDirA     , OUTPUT );
    pinMode( pinStepA    , OUTPUT );
    pinMode( pinDirB     , OUTPUT );
    pinMode( pinStepB    , OUTPUT );
    pinMode( PING        , OUTPUT );
    pinMode( PING_MASTER , INPUT  );

	Serial.begin(9600);
	Serial.println("Booting up...");
    Serial.println(" ");
    Serial.println("_ Astromech Industrie _");
    Serial.print("  BB-8 Version ");
    Serial.println(version);
    Serial.println("===================");
    Serial.println(" ");
    


    
    Serial.print("Initializing I2C devices...  I2C ADRESS :");
    Serial.print(i2cAdress);
  
    Wire.begin(i2cAdress);
    
    accelgyro.initialize();
	Serial.println(" [ OK ]");

    Serial.print("Connect to the MPU6050 : ");
    Serial.println(accelgyro.testConnection());
    //Serial.println(accelgyro.testConnection() ? "Connection successful" : "Connection failed");
    
    Serial.println("Initializing RF24 devices...  ");
    radio.begin();
    radio.openWritingPipe(pipe);
    radio.startListening();
    
    Serial.println("Booting successful !");
    Serial.println("====================");
	
}

void loop()
{
	/* SEND
	String theMessage = "Hello there!";
	int messageSize = theMessage.length();
	for (int i = 0; i < messageSize; i++) 
	{
    	int charToSend[1];
    	charToSend[0] = theMessage.charAt(i);
    	radio.write(charToSend,1);
  	}  
 
	msg[0] = 2; 
    radio.write(msg,1);
    */
    
    if (radio.available())
    {
    	bool done = false;  
    	done = radio.read(msg, 1); 
    	char theChar = msg[0];
      	if (msg[0] != 2)
      	{
        	theMessage.concat(theChar);
        }
      	else 
      	{
       		Serial.println(theMessage);

			int str_len = theMessage.length() + 1; 
 
			char msg[str_len];

			// Copy it over 
			theMessage.toCharArray(msg, str_len);
       		
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
          			//	case 'x': setpointX  = val; break;
          			//	case 'y': setpointY  = val; break;
      	  			//	case 'm': multiplier = val; break;
      				//	case 'i': invertXY   = val; break;
      				//	case 'j': invertX    = val; break;
      				//	case 'k': invertX    = val; break;
      				//	case 'h': hysteresis = val; break;
        			}
      			}
    		}
       		theMessage= ""; 
      	}
   }
  
}
