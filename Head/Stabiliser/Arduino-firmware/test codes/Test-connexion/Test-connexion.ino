int a, b, x, y, z, s ;

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
    char msg[30];
  if (Serial.available() > 0)
  {
    Serial.readBytesUntil('\n', msg, sizeof msg);

    if (strcmp(strtok(msg, " "), "RUN") == 0)
    {
      // trouvé le message
      char *p;
      while ((p = strtok(NULL, " ")) != NULL)
      {
        int val = atoi(p + 1);
        switch (*p)
        {
          case 'a': a = val; break;
          case 'b': b = val; break;
          case 'x': x = val; break;
          case 'y': y = val; break;
          case 'z': z = val; break;
          case 's': s = val; break;
        }
      }
    }

    
    Serial.print(a);
    Serial.print("-");
    Serial.print(b);
    Serial.print("-");
    Serial.print(x);
    Serial.print("-");
    Serial.print(y);
    Serial.print("-");
    Serial.print(z);
    Serial.print("-");
    Serial.println(s);
  }

  delay(1000);
  Serial.println("ping");
}
