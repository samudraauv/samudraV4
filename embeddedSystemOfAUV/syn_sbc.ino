void syn_sbc(){
  #if gui
  if(!handshake)
  {
    bool one=false;
    while(Serial.read()!='h' && handshake==false)
    {
      if(!one)
      {
        Serial.write('s');
        one=true;
      }
      // do nothing
      // wait for the handshake
    }
    if(!handshake)
    {
      handshake=true;
      led_status(LOW, LOW, HIGH);
    }
  }
  #endif
}

