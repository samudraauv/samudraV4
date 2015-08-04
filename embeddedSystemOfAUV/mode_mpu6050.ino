void mode()
{
  /*

   Register 26 – Configuration
   CONFIG
   Type: Read/Write

   Register (Hex) 	Register
   (Decimal) 	Bit7 	Bit6 	Bit5 	Bit4 	Bit3 	Bit2 	Bit1 	Bit0
   1A 	                    26  	 - 	 - 	 EXT_SYNC_SET[2:0] 	  DLPF_CFG[2:0]


   DLPF_CFG 	  Accelerometer                                   Gyroscope
   (Fs = 1kHz)
   Bandwidth (Hz)     Delay (ms) 	Bandwidth (Hz) 	Delay (ms) 	Fs (kHz)
   0 	    260 	          0 	            256 	    0.98 	8
   1 	    184       	        2.0 	            188 	    1.9 	1
   2 	    94 	                3.0 	            98 	            2.8 	1
   3 	    44           	4.9 	            42 	            4.8 	1
   4 	    21             	8.5 	            20 	            8.3 	1
   5 	    10       	        13.8           	    10 	            13.4 	1
   6 	    5 	                19.0 	            5 	            18.6 	1
   7 	          RESERVED 	                        RESERVED 	        8

   Register 27 – Gyroscope Configuration
   GYRO_CONFIG
   Type: Read/Write

   Register (Hex) 	Register
   (Decimal) 	Bit7 	  Bit6 	  Bit5   	Bit4 	Bit3 	    Bit2 	Bit1 	Bit0
   1B 	                  27 	        XG_ST 	  YG_ST   ZG_ST 	FS_SEL[1:0] 	      - 	- 	-


   FS_SEL 	  Full Scale Range
   0 	  ± 250 °/s
   1 	  ± 500 °/s
   2 	  ± 1000 °/s
   3 	  ± 2000 °/s

   Register 28 – Accelerometer Configuration
   ACCEL_CONFIG
   Type: Read/Write
   Register (Hex) 	Register
   (Decimal) 	Bit7 	Bit6 	Bit5 	Bit4 	Bit3 	Bit2 	Bit1 	Bit0
   1C 	                   28   	XA_ST 	YA_ST 	ZA_ST 	AFS_SEL[1:0] 	-       -        -

   AFS_SEL 	Full Scale Range
   0 	        ± 2g
   1 	        ± 4g
   2 	        ± 8g
   3 	        ± 16g


   Register 107 – Power Management 1
   PWR_MGMT_1
   Type: Read/Write
   Register (Hex)   	Register
   (Decimal) 	Bit7     	Bit6 	Bit5 	Bit4 	Bit3 	    Bit2     Bit1     Bit0
   6B 	                    107 	DEVICE _RESET 	SLEEP 	CYCLE 	 - 	TEMP_DIS          CLKSEL[2:0]


   */
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  i2cData[0] = 7;
  i2cData[1] = 0x05;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  unsigned long int timer_local = millis();
  while (millis() - timer_local < 1000) {
    bool a=i2cWrite(MPU6050, 0x19, i2cData, 4, false);
    if (a)
    {
      mpu = false;
    }
    else
    {
      mpu = true;
      while (i2cWrite(MPU6050, 0x6B, 0x01, true));
      while (i2cRead(MPU6050, 0x75, i2cData, 1));
       if (i2cData[0] != 0x68) {

      mpu = false;
      //while (1);
    }
    else {
      mpu = true;
    }
      timer_local -= 1000;
    }
   
  }


  //stop : boolean. true will send a stop message, releasing the bus after transmission. false will send a restart, keeping the connect
  //on active.

}




