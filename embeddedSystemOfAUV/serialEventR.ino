void serialEventR()
{
  int data_length=12;
  if(Serial.available()>data_length-1&&zero_ref)
  {
    bool correctFrame=true;
    int b=Serial.read();
    if(b=='$')
    {
      uint32_t timeFileOut=millis();
      data="";
      char c= Serial.read();
      while(c!='%')
      {
        if(c=='-1')
        {

        }
        else 
        {
          data +=c;
          timeFileOut=millis();
        }
        //Serial.println(c);
        while(!Serial.available()&&(millis()-timeFileOut<2));
       
       if(Serial.available())
      {
         c= Serial.read();
      } 
      else
      {
        correctFrame=false;
        c='%';
      }
      }

      //Serial.println(data);
      if(correctFrame)
      command(data);
    }
  }
}
void command(String pie)
{
  //file=true;
  //file_scheduler=millis();
  String p[3]="";
  int j=0;
  for(int i=0;i<pie.length();i++)
  {
    if(pie[i]==',')
    {
      j++;
    }
    else
    {
      p[j]+=pie[i];
    }
  }
  //Serial.println(p[0]);
 // Serial.println(p[1]);
 // Serial.println(p[2]);
  
  if(p[0][0]=='u'&&j>=0)
  {
    int u;
    if(j>0)
      u=(p[1].substring(0)).toInt();
    else
      u=50;
 
    u=constrain(u,0,100);
    if(u>50)
    {
      u=map(u,51,100,1,50);
    }
    else if(u<50)
    {
      u=map(u,0,49,-50,1);
    }
    else
    {
      u=0;
    }
    altError=u;
    //Serial.println(u);
    
  }
  
  else 
  {
     
  }
  
  
  if(p[0][1]=='l'&&j>=0)
  {
    int l;
    if(j>1)
      l=(p[2].substring(0)).toInt();
    else
      l=50;
  
    l=constrain(l,0,100);
    if(l>50)
    {
      l=map(l,51,100,1,50);
    }
    else if(l<50)
    {
      l=map(l,0,49,-50,1);
    }
    else
    {
      l=0;
    }
   yawError=l;
   //Serial.println(l);
  }
  else
  {
   
  }
  if(p[0][2]=='f'&&j>=0)
  {
    file=true;
    file_scheduler=millis();
   }
   else
   {
 
   }
   
}
void acknowledge()
{
  if(zero_ref)
  {

  if(handshake){
    int k=1;
    if(millis()-file_scheduler>1000)
    {
      file=false;
      k=0;
    }
    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print('$');
    Serial.print(',');
    Serial.print(k);
    Serial.print(',');
    Serial.print(temperature);
    Serial.print(",");
    Serial.println("%"); 
  }
  }
}

