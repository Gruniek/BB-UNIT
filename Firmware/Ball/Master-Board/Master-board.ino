String version = "0.1.1";
/*
http://www.electroschematics.com/9351/arduino-digital-voltmeter/

 >      FIRMWARE ONLY FOR AN ARDUINO NANO !!    <
 > IF YOU USE ANOTHER ARDUINO, ADAPT THE PINOUT <
 
 ###################################################
  ______  ______     _     _ ______  _____ _______ 
 (____  \(____  \   | |   | |  ___ \(_____|_______)
  ____)  )____)  )__| |   | | |   | | | |    | |         
 |  __  (|  __  (___) |   | | |   | | | |    | |      
 | |__)  ) |__)  )  | |___| | |   | |_| |_   | | 
 |______/|______/    \______|_|   |_(_____)  |_|
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
#include "PCF8574.h"

int i2cAdress      = 1;

int pinDirA        = 2;      // PIN DIRECTION FOR X 
int pinStepA       = 3;      // PIN STEP FOR X
int pinEnable      = 4;      // Activation off all stepper

int pinDirB        = 5;      // PIN DIRECTION FOR Y
int pinStepB       = 6;      // PIN STEP FOR Y

int CurrentBat1    = 0;
int CurrentBat2    = 1;
int CurrentBat3    = 2;
int CurrentBat4    = 3;
int Pinvoltage     = 6;

int PING           = 10; 
int PING_SLAVE     = 0;  // A0
int pingMaster     = 0;
bool LED_PING      = false;
bool tmpPing       = false;
bool tmpPing2      = false;
bool ifPing        = false;
bool emgStop       = false;
bool production    = false;

unsigned long trigSec  = 1000;
unsigned long trigS    = 0;

/** PCF8574 instance */
PCF8574 expander;

//RF24 PINOUT
//
// RF24 -- ARDUINO
//    1 -- GND
//    2 -- 3,3V
//    3 -- 8
//    4 -- 7
//    5 -- 13
//    6 -- 11
//    7 -- 12
//    8 -- 2

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
    
    /* Start I2C bus and PCF8574 instance */
  	expander.begin(0x20);
  
  	/* Setup some PCF8574 pins for demo */
  	expander.pinMode(0, OUTPUT);
  	expander.pinMode(1, OUTPUT);
  	expander.pinMode(2, OUTPUT);
    expander.pinMode(3, OUTPUT);
    expander.pinMode(4, OUTPUT);
    expander.pinMode(5, OUTPUT);
    expander.pinMode(6, OUTPUT);
    expander.pinMode(7, OUTPUT);
    
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
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
     if((analogRead(PING_SLAVE) > 200) && ifPing) ifPing = false;
  
  
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
    }
  
}






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
