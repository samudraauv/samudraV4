import processing.serial.*; //import the Serial library
//this library is being used for communication with arduino

Serial arduino; // instance of serial class for arduino

boolean handshake=false;

//boolean z=false;
//int last;

//file reading
BufferedReader reader;
String line;

//Image and Font Loading
PImage ocean;
PFont text_font;
PFont text_font2;

//input from arduino
//this variables are updated by frame return by arduino
float battery_level=50.00; // this variable takes battery level of thrusters battery from arduino
float temp_value; // this varible displays temperature from MPU6050
int state; //it displays state of running of arduino board


String message; ///mesaage to be printed in state textbox is storeed in this variable
int eprom_temp;
int eprom_code;

//minutes seconds 
int min;
int sec=0;
int i=0; //dummy variable



//---NOT BEING USED-----
//calibration part
boolean box=false;
int textbox=0;
String text[];
float constants[];
boolean uploading;
int uploaded=0;


//serial communication 
String serial_data_from_arduino;
//String[] dataz;
volatile boolean ack=false;

long timer_file; //this variable is used for sending file data to arduino at constant rate
int timeOut; // this variable is used to detect connection breaking of with arduino

void setup()
{

  size(displayWidth-600, displayHeight-100); //size(w, h, renderer)

  ocean=loadImage("ocean.jpg");
  text_font=loadFont("ForteMT-48.vlw");
  text_font2=loadFont("AgencyFB-Reg-48.vlw");

  frameRate(120);

  //text and constants are used of taking kp, ki, kd values from user and sending it to arduino
  //these are not being used in samudra v4
  text= new String[13];
  constants = new float[13];
  for (int i=0; i<13; i++)
  {
    text[i]=" ";
  }

  //String listS=Serial.list()[0];
  arduino=new Serial(this, "COM24", 115200);


  serial_data_from_arduino = arduino.readStringUntil(10);

  //file being opened
  reader = createReader("opencv.txt");

  state=1;


  message="File Created";


  timer_file=millis();
  timeOut=millis();
}


void draw()
{
  ocean.resize(width, height);
  background(ocean);
  textFont(text_font);



  //basic functions
  header();
  battery(battery_level);
  temp(temp_value);
  status(message, state);
  eprom(eprom_temp, eprom_code);


  //displays time
  timer();


  //calibration part which is not being used
  box=false;
  textbox=input_box(width/15, height/1.6, 1, "Kp_Rate_R");
  textbox=input_box(width/15, height/1.4, 2, "Ki_Rate_R");
  textbox=input_box(width/15, height/1.25, 3, "Kd_Rate_R");
  textbox=input_box(width/5, height/1.6, 4, "Kp_Rate_P");
  textbox=input_box(width/5, height/1.4, 5, "Ki_Rate_P");
  textbox=input_box(width/5, height/1.25, 6, "Kd_Rate_P");
  textbox=input_box(width/2.92, height/1.6, 7, "Kp_Rate_Y");
  textbox=input_box(width/2.92, height/1.4, 8, "Ki_Rate_Y");
  textbox=input_box(width/2.92, height/1.25, 9, "Kd_Rate_Y");
  textbox=input_box(width/2, height/1.6, 10, "Kp_Ang_R");
  textbox=input_box(width/2, height/1.4, 11, "Ki_Ang_P");
  textbox=input_box(width/2, height/1.25, 12, "Kd_Ang_Y");

  if (mousePressed&&!box)
  {
    textbox=0;
  }
  upload(uploaded);
  proc_to_ard();
  file_to_proc();

  pointer();
  //println(constants[textbox]);
}

void keyPressed()
{
  if (key!=CODED&&(key=='1'||key=='2'||key=='3'||key=='4'||key=='5'||key=='6'||key=='7'||key=='8'||key=='9'||key=='0'||key=='.'))
  {
    if (text[textbox].length()<5&&textbox!=0)
    {
      text[textbox]+=str(key);
      if (Float.isNaN(float(text[textbox])))
      {
        text[textbox]=text[textbox].substring(0, Math.max(0, text[textbox].length()-1));
      }
      if (float(text[textbox])>99.9)
      {
        text[textbox]=text[textbox].substring(0, Math.max(0, text[textbox].length()-1));
      }
      if (Float.isNaN(float(text[textbox])))
      {
        text[textbox]+='0';
        text[textbox]+='.';
      }
      constants[textbox]=float(text[textbox]);
    }
  } else if (keyCode==BACKSPACE)
  {
    if (textbox!=0)
    {
      text[textbox]=text[textbox].substring(0, Math.max(0, text[textbox].length()-1));
      if (Float.isNaN(float(text[textbox]) ))
      {
        text[textbox]=text[textbox].substring(0, Math.max(0, text[textbox].length()-1));
      }
      if (Float.isNaN(float(text[textbox])))
      {
        text[textbox]+='0';
        text[textbox]+='.';
      }
      constants[textbox]=float(text[textbox]);
    }
  }
  if (key == ENTER&&textbox!=0&&textbox<12) 
  {
    textbox=textbox+1;
  }
}

void serialEvent(Serial arduino) 
{
  //read String from the serial port:
  if (handshake==false)
  {
    int inString = arduino.read();
    if (inString=='s')
    {
      handshake=true;
      ack=true;
      arduino.write('h');
      message="Arduino Connected";
      state=2;
    }
    //println(inString);
  } else
  {

    serial_data_from_arduino = arduino.readStringUntil(10);
    if (serial_data_from_arduino != null) 
    {  
      //println(serial_data_from_arduino);
      String[] a = split(serial_data_from_arduino, ",");
      if (a[0].equals("$"))
      {
        if (a[1].equals("1"))
        {
          ack=true;
          message="Arduino Syn with File";
          state=3;
          //println(a[2]);
        } else
        {
          ack=false;
          message="Arduino NACK";
          state=11;
        }
        timeOut=millis();
        temp_value=int(a[2]);
      }
      //println(a[3]);

      serial_data_from_arduino=null;
    }
  }
}

