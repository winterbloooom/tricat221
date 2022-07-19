void autonomous(){
  nh.spinOnce();  //AC
  delay(1);

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
