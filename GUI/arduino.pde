void proc_to_ard()
{
  if (millis()-timeOut>20000)
  {
    ack=false;
    println("Error in Arduino");
    message="Arduino Comm Error";
    state=12;
  }
}

void file_to_proc()
{
  String[] stuff = loadStrings("opencv.txt");
  /*String stuff;
  try {
    stuff = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    stuff = null;
  }
  */
  if (stuff==null)
  {
    message="Empty File";
    state=13;
  } 
  else if (handshake==true&&(millis()-timer_file)>15)
  {
    //String[] b = split(stuff[0], ",");
    int len=0;
    boolean correct=true;
    try 
    {
     len=stuff[0].length();
     } 
  catch (Exception e) 
  {
   //println("fuck");
    //stuff[0] = null;
    correct=false;
  }
   
    if(correct==true)
    {
    for (int i=0; i<len; i++)
    {
      arduino.write(stuff[0].charAt(i));
    }
    //println(stuff[0].charAt(0));
    timer_file=millis();
    }
  } 
  else
  {
    reader = createReader("opencv.txt");
  }
}

