void led_status(bool r, bool y, bool b) {
  digitalWrite(red_led, y);
  digitalWrite(yellow_led, r);
  digitalWrite(blue_led, b);

}

void led_display(bool f)
{
  if(handshake&&zero_ref)
  {
    if(f==true)
    {
      led_status(LOW, LOW, HIGH);
    }
    else
    {
      led_status(LOW,HIGH,HIGH);
    }
  }
}


