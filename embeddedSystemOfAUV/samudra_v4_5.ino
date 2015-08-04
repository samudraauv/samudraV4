//COPYRIGHTS @SAMUDRA-AIT AUV version4 2015
/*
For roll, clockwise is negative and for pitch, counterclockwise is negative.
For yaw, counter clockwise is negative.
------GUI MODE------
In GUI Mode, the hanshaking and file reading procesdure is enabled.
Two global variables handshake and file are used to store status of these procedure.
First, after start of SAMUDRA imu, lcd is intialized. Stuiable message is printed on LCD like "SAMUDRA AIT AUV v4". Motors is being mounted using function like motorArm()(i,e, pinMode, Direction of
motor is being set ther);
lcd_status(mpu, hmc, file); and led_status are always executed from th start of arduino to it's end(shutdown/poweroff). However, lcd_status's exection rate is restricted by
AUV"S scheduler using lcd_scheduler global variable. led_status is executed every time it is being called.
U CAN CHANGE RATE OF SSCHEDULER AT THE PLACE OF THEIR CAPOMARSION WHICH CURRENT TIME(from millis()) like if(millis()-lcd_scheduler>RATE_OF_EXECUTION)

setAttitudeAndRate(); is the most important function because it calculates setpoints and also it cahnges state of glocbal variable zero_ref after the calculation of setpoint
First, it gets executed only in time frame 4 to 7 sc from the start or arduino. Then after 7th sec, it changes gloabla variabler zero_ref from FALSE to TRUE indicationg the end of calculation 
of setpoint. After that pid function are reset and hanshake procedure is initialted in setAttitudeAndRate() itself.
Afer 7sec, setAttitudeAndRate() cannott execute as zero_Ref is true then(check implementation of this function)
after 7sec, NOW PID COMPUTE FUNCTION CAN GET EXECUTED AND MOTOR WILL START AFTER zero_ref GETS TRUE

acknowledge();, serialEventR();, controller(); functions are executed after zero_ref becomes true.
acknowledge(); sends data from arduino to gui after every 1 sec and there is one ack byte in ack frame which tells that arduino is able to read file or not. 
serialEventR(); is executed every 10ms to read data from aprocessinf which is file's data
controller() function contains motor driving functions and pid function.

-------DEBUG MODE-------
It is similar to gui mode but no file reading and acknowledge() function is executed.
global variables file and handshake are always true in this mode by default.
u can use this mode for printing angles, pid output, magnetic field etc.

NOTE THAT debug and gui macros CANNOT BE SAME AT THE ASME TIME. THEY ARE OF COMPLEMEMNARY STATE.


*/
//LIBRARIES
#include <LiquidCrystal.h>//LCD Library
#include <Wire.h>//I2C library used to read values from sensors like hmc5883l and mpu6050
#include "Kalman.h"//kalman filtering to fuse data from gyroscope and acceleromter and to output noise free angles
#define VERSION "SAMUDRA v4_5" //current version of codes of samudra ait-auv v4 2015



//DEBUGGING SECTION
//this section is used in debugging mode when values from sensors and pid block is to be print on terminal in debugging mode
#define temp 0// 1 for priting temp values
#define angle 1 // 1 for printing angles
#define pid 0 // 1 for printing pid output both rate and angle pid
#define gyro 1// 1 for prinitng gyrocope values
#define acc 0// 1 for printing acceleration
#define magnet 0// 1 for prinitng magnetic field in x, y and z direction
#define mag_cal 0// 1 for printinfg calibrated magnetic constants
#define zero_reference 0// 1 for prinintg setpoint
#define file_debug 0

#define LCD_BACKLIGHT 53  //Hitachi HD44780 driver

//MODES OF SAMUDRA
#define debug false //true for debuging mode
//in this mode file will not get read. It is used only with debugging section to print values of sensors and pid block output
#define lcd_enable true //it should be always on as it displays values on LCD
#define gui true //it is TRUE when to read data from file, handshake

//NOTE- debug and gui macro cannot be true at the same time 

//FILE SECTION
String data;
double yawError=0.00;
double altError=0.00;


//STATUS VARIABLES AND SBC COMMUNICATION
bool mpu = false; //gets true when it detects mpu6050 connected
bool hmc = false; //gets true when it detects hmc5883l connected

//this section disable handshake and file reading when gui mose is false or disabled
#if gui
bool file = false;
bool handshake = false;
#else
bool file = true;
bool handshake = true;
#endif

//STATUS LEDS
#define red_led 19
#define blue_led 18
#define yellow_led 17


//IMU VARIABLES
Kalman kalmanX, kalmanY, kalmanZ;
const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E;

double accX1, accY1, accZ1;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroRoll = 0.0;
double gyroPitch = 0.0;
double gyroYaw = 0.0;
double magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw;

double gyroXangle, gyroYangle, gyroZangle;
double kalRoll, kalPitch, kalYaw;

uint32_t timer;
uint8_t i2cData[14];

#define MAG0MAX 603
#define MAG0MIN -578 //
#define MAG1MAX 542
#define MAG1MIN -701 //
#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = {
  (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2
};
double magGain[3];



//SETPOINT or ZERO REFERENCE
double zero_angle_roll;
double zero_angle_pitch;
double zero_angle_yaw;
double zero_rate_roll;
double zero_rate_pitch;
double zero_rate_yaw;
boolean zero_ref = false; //becomes true when zero_ref of gyro, kalman angles are calculated

//COMPLEMENTARY FILTER VALUES
//u can set the values of this variables from 0.80 to 0.97.
//higher the value, greater would be noise reduction effieciency from senosrs output 
double compAngleSet = 0.95;
double compRateSet = 0.95;
double compGyroFilter = 0.90;
double compAcc = 0.90;

//LiquidCrystal(rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7);
LiquidCrystal lcd(31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51);
String lcd_print = "samudra winner niot 2015";


//AUV SCHEDULER
//it makes sure that certain function are executed at some constant rate
//they only stores last time of execution. However, the rate can be changed at the place of their use 
uint32_t lcd_scheduler;
uint32_t file_scheduler;
uint32_t ack_scheduler;
uint32_t timer_sp; //setpoint timer


//RESET FUNCTION
//this funnction is not being used in current version of samudra
void(* resetFunc) (void) = 0;

//MOTOR AND MOTOR DRIVER SECTION
int l_pwm = 2; //1
int r_pwm = 3; //2
int lf_pwm = 5; //3
int rf_pwm = 6; //4
int lb_pwm = 7; //5
int rb_pwm = 8; //6

int l_dir = 4;
int r_dir = 9;
int lf_dir = 10;
int rf_dir = 11;
int lb_dir = 12;
int rb_dir = 13;

//FORWARD SPEED AND DROWNING SPEED(b_speed)
unsigned long int f_speed;
unsigned long int b_speed;
unsigned long int threshold=150;
unsigned long int maximun_pwm=510; //255 for normal



//END--------------------------------------------------------------------------------------------------------------------------------

//PID_ATLAB CLASS
class PID_ATLAB
{

  public:

    PID_ATLAB(double, double, double, double = -500, double = 500);

    double Compute(double input, double zero, double Setpoint = 0.00);

    double ComputeFixedHz(double input, double zero, double sampletime, double Setpoint = 0.00);

    void SetOutputLimits(double, double);

    void SetTunings(double, double, double);
    void reset();

  private:

    double kp;
    double ki;
    double kd;
    int controllerDirection;

    unsigned long lastTime;
    double ITerm, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    double output;

    double error;
    double SampleTimeInSec;
    double dInput;

};


//ROLL RATE PID
double p_rol_rate = 0;
double i_rol_rate = 0;
double d_rol_rate = 0;

PID_ATLAB PIDrol_rate(p_rol_rate , i_rol_rate , d_rol_rate);
double out_rate_x;


//PITCH RATE PID
double p_pitch_rate = 0;
double i_pitch_rate = 0;
double d_pitch_rate = 0;

PID_ATLAB PIDpitch_rate(p_pitch_rate , i_pitch_rate , d_pitch_rate);
double out_rate_y;


//YAW RATE PID
double p_yaw_rate = 0;
double i_yaw_rate = 0;
double d_yaw_rate = 0;

PID_ATLAB PIDyaw_rate(p_yaw_rate , i_yaw_rate , d_yaw_rate);
double out_rate_z;


//ROLL ANGLE STABILIZE PID
double p_rol_angle = 0;
double i_rol_angle = 0;
double d_rol_angle = 0;

PID_ATLAB PIDrol_angle(p_rol_angle , i_rol_angle , d_rol_angle);
double out_angle_x;


//PITCH ANGLE STABILIZE PID
double p_pitch_angle = 0;
double i_pitch_angle = 0;
double d_pitch_angle = 0;
PID_ATLAB PIDpitch_angle(p_pitch_angle , i_pitch_angle , d_pitch_angle);
double out_angle_y;

//YAW ANGLE STABILIZE PID
double p_yaw_angle = 0;
double i_yaw_angle = 0;
double d_yaw_angle = 0;
PID_ATLAB PIDyaw_angle(p_yaw_angle , i_yaw_angle , d_yaw_angle);
double out_angle_z;

//ALTITUDE STABILIZATION PID
double p_alt = 0;
double i_alt = 0;
double d_alt = 0;
PID_ATLAB PIDalt(p_alt , i_alt , d_alt);
double out_alt;

//PID AND CASCADING SECTION
//rate pid is always on. However, angle pid block can be switched on or off usinf variable pid_angle
//pid_rate_angle is used for cascading angle pid output to rate pid input IF it is true
boolean pid_rate_angle = false;
boolean pid_angle = true;
boolean pid_alt=true;

void setup() {


  delay(100);
  Serial.begin(115200);//buad rate for serial terminal and gui communication
  Wire.begin();

  //LCD INITIALIZATION
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  lcd.begin(16, 2);

  //led_status(red, yellow, blue);
  led_status(LOW, LOW, LOW);

#if debug
  Serial.println();
  Serial.println(VERSION);
#endif

#if lcd_enable
  lcd.setCursor(3, 0);
  lcd.print(VERSION);
  lcd.setCursor(3, 1);
  lcd.print("AIACTR-AUV");
#endif

   //MPU6050 and HMC5883l is getting initialize here
  imuInitialize();

  //LED pins are geting initialize
  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(blue_led, OUTPUT);

  //this function is executred only once
  //direction of each motor can be changed here by changing digitalWrite(dir,HIGH/LOW);
  motorArm();

  delay(1000);

  //LCD FIRST PRINTING
  lcd_status(mpu, hmc, file);
  //lcd_function(lcd_print);

  //timer initializer
  timer_sp = millis();
  lcd_scheduler = millis();
  ack_scheduler = millis();
  file_scheduler = millis();


}

void loop() {

  //LCD SCHEDULER (all functions are executed at cinstant rate)
  if (millis() - lcd_scheduler > 2000) {
    lcd_status(mpu, hmc, file);
    //lcd_display(lcd_print); //lcd_print is a string
    if (file && hmc && mpu && handshake)
      lcd_backlight_enable(LOW);
    lcd_scheduler = millis();
  }

  imu();


 //setpoint calculation function
 
  setAttitudeAndRate();
  
  #if gui
  if (millis() - ack_scheduler>1000)
  {
    //ack is always output from arduino in gui mode even processing is not sending any data to it
    //however, if file is being send by processing from more than 2 sec, arduino changes 0 in place of ack bit
    //this bit is used by processing to display message like "ARDUINO NACK"
    //means arduino is unable to read the file as it is not being send to it by processing itslef
    acknowledge();
    ack_scheduler=millis();
  }
  if (millis() - file_scheduler>15)
  {
    //file is being read at every 10milli sec
    serialEventR(); 
  }
  #endif
  
  //it drives the motors and pid Compute function of angle pid and rate pud are being executred here
  controller();
  led_display(file);

}



