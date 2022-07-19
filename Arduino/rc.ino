void rc(){
  //Serial.begin(9600);
  ch1 = pulseIn(ip1, HIGH); //THRUSTER

  if (ch1 > 1550)
  {
    val1=map(ch1, 1550, 1900, 1550, 1900); 
    thruster1.writeMicroseconds(val1);
    thruster2.writeMicroseconds(val1);
    delay(100);

  }
  else if (ch1 < 1450)
  {
    val1=map(ch1, 1450, 1100, 1450, 1300);
    thruster1.writeMicroseconds(val1);
    thruster2.writeMicroseconds(val1);
    delay(100);
    }
  else if (1450< ch1 <1550)
  {
    
    thruster1.writeMicroseconds(1500);
    thruster2.writeMicroseconds(1500);
    delay(100);
    }
    
  ch2 = pulseIn(ip2, HIGH); // SERVO
  
  if (1100<ch2<1900)
  {
    val2 = map(ch2, 1900, 1100, 95-14, 95+14);  //중앙값 93 +- 13
    servo1.write(val2);
    servo2.write(val2 - 2); // 1번(왼) 2번(오) 차이 값 작아지는게 우회전
    delay(100);
    
  }
/*
  if (1100< ch2 < 1450)
  {
    val2 = map(ch2, 1450, 1000, 94, 70);
    servo1.write(val2);
    servo2.write(val2 - 2);
    delay(100);
  }
  else if ( 1450< ch2 < 1550)
  {
    servo1.write(94);
    servo2.write(92);
    delay(100);
  }
  else if ( 1550< ch2 <1900)
  {
    val2 = map(ch2, 1900, 1550, 118, 94);
    servo1.write(val2);
    servo2.write(val2-2);
    delay(100);
  }*/

  ch6 = pulseIn(ip6,HIGH); //RELAY
  { 
      if (ch6 < 1450)
      {
      digitalWrite(relaypin,LOW);
      delay(100);
      }
      else if (ch6 > 1450)
      {
      digitalWrite(relaypin,HIGH);
      delay(100);
      }
  }
  }
