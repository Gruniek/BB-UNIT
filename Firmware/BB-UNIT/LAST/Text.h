#include <Arduino.h>

void LOGO_TEXT()
{
	
  // Blabla, blablabla
  Serial.println("==================================================");
  Serial.println("  _______ ______     _     _  ______  ____ ______ ");
  Serial.println(" (____   (____  )   | |   | |/ ___  |(____|______)");
  Serial.println("  ____)  )____)  )__| |   | | |   | | | |   | |   ");
  Serial.println(" |  __  (|  __  (___) |   | | |   | | | |   | |   ");
  Serial.println(" | |__)  ) |__)  )  | |___| | |   | |_| |_  | |   ");
  Serial.println(" |______/|______/   |______/|_|   |_(_____) |_|   ");
  Serial.println("==================================================");
}

void HELP_TEXT()
{
  Serial.println(" ");
  Serial.println("- HELP - ");
  Serial.println(" BB-CODE : COMMAND ");
  Serial.println(" > RUN   : To Run de process");
  Serial.println(" > STOP  : To Stop the process");
  Serial.println(" > RESET : For reinitialize the system");
  Serial.println(" > DIE   : Do a physical reset");
  Serial.println(" > SET   : For change somes parameters");
  Serial.println("  --> SET x87 y97 h1");
  Serial.println(" > TEST  : For done a fonctional testing of the hardware");
  Serial.println(" > ROTATE");
  Serial.println(" > ENABLE");
  Serial.println(" > MOVE");
}

void COPY_TEXT()
{
  Serial.println("                                                  ");
  Serial.println("            _ Astromech Industrie _               ");
  Serial.println("          (c)Lucas Film - (c)Disney               ");
  Serial.println("                                                  ");
  Serial.println("      > https://github.com/Gruniek/BB-UNIT <      ");
  Serial.println("           > http://r2builders.fr/ <              ");
  Serial.println("                                                  ");
  Serial.println("           > Code by Daniel Mostosi <             ");
  Serial.println("        Please respect the code. Thanks           ");
  Serial.println("                Report an issues?                 ");
  Serial.println("    https://github.com/Gruniek/BB-UNIT/issues     ");
  Serial.println("                                                  ");
}