//have a look at last two functions only as only these functions are important to you
//changing anything in this tab can give unpredicted results

void imuInitialize() {
  mode();


  unsigned long int timer_local = millis();
  while (millis() - timer_local < 1000) {
    bool a=i2cWrite(HMC5883L, 0x02, 0x00, true);
    if (a)
    {
      hmc = false;
#if debug
      Serial.print("STATUS - ");
      Serial.println(-1);
#endif
    }
    else
    {
      hmc = true;
      if (mpu == true)
      {
#if debug
        Serial.print("STATUS - ");
        Serial.println(1);
#endif
        led_status(HIGH, LOW, LOW);

      }
      else
      {
#if debug
        Serial.print("STATUS - ");
        Serial.println(-1);
#endif
      }
      timer_local -= 1000;
    }

  }
  //lcd_function(lcd_print+" working");

  //while ();
  if (hmc == true) {
    //calibrateMag();
    i2cWrite(HMC5883L, 0x00, 0x11, true);
    delay(100);
    //updateHMC5883L();

    while (i2cRead(HMC5883L, 0x03, i2cData, 6));
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);

    int16_t magPosOff[3] = {
      magX, magY, magZ
    };

    i2cWrite(HMC5883L, 0x00, 0x12, true);
    delay(100);
    //updateHMC5883L();

    while (i2cRead(HMC5883L, 0x03, i2cData, 6));
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);

    int16_t magNegOff[3] = {
      magX, magY, magZ
    };

    i2cWrite(HMC5883L, 0x00, 0x10, true);

    magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

    delay(100);

    while (i2cRead(HMC5883L, 0x03, i2cData, 6));
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);
#if debug
#if mag_cal
    Serial.print("Mag cal: ");
    Serial.print(magNegOff[0] - magPosOff[0]);
    Serial.print(",");
    Serial.print(magNegOff[1] - magPosOff[1]);
    Serial.print(",");
    Serial.println(magNegOff[2] - magPosOff[2]);

    Serial.print("Gain: ");
    Serial.print(magGain[0]);
    Serial.print(",");
    Serial.print(magGain[1]);
    Serial.print(",");
    Serial.println(magGain[2]);
#endif
#endif
  }
  /* Set Kalman and gyro starting angle */
  //updateMPU6050();
  if (mpu == true)
  {
    while (i2cRead(MPU6050, 0x3B, i2cData, 14));
    accX1 = ((i2cData[0] << 8) | i2cData[1]);
    accY1 = -((i2cData[2] << 8) | i2cData[3]);
    accZ1 = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = -(i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = -(i2cData[12] << 8) | i2cData[13];
  }
  //updateHMC5883L();
  /*if(hmc==true)
   {
   while (i2cRead(HMC5883L, 0x03, i2cData, 6));
   magX = ((i2cData[0] << 8) | i2cData[1]);
   magZ = ((i2cData[2] << 8) | i2cData[3]);
   magY = ((i2cData[4] << 8) | i2cData[5]);
   }
   */
  //updatePitchRoll();


  roll = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  pitch = atan2(-accX1, accZ1) * RAD_TO_DEG;

  //updateYaw();
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  magX *= magGain[0];
  magY *= magGain[1];
  magZ *= magGain[2];

  magX -= magOffset[0];
  magY -= magOffset[1];
  magZ -= magOffset[2];

  double rollAngle = kalRoll * DEG_TO_RAD;
  double pitchAngle = kalPitch * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

  yaw *= -1;

  kalmanX.setAngle(roll);
  gyroXangle = roll;

  kalmanY.setAngle(pitch);
  gyroYangle = pitch;

  kalmanZ.setAngle(yaw);
  gyroZangle = yaw;



  timer = micros();
}

void imu() {
  //updateMPU6050();

  if(mpu==true)
  {
    while (i2cRead(MPU6050, 0x3B, i2cData, 14));
    accX1 = ((i2cData[0] << 8) | i2cData[1]);
    accY1 = -((i2cData[2] << 8) | i2cData[3]);
    accZ1 = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = -(i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = -(i2cData[12] << 8) | i2cData[13];
  }
  accX = (compAcc * accX + (1 - compAcc) * (accX1)) / 100.00;
  accY = (compAcc * accY + (1 - compAcc) * (accY1)) / 100.00;
  accZ = (compAcc * accZ + (1 - compAcc) * (accZ1)) / 100.00;

  //updateHMC5883L();
  if(hmc==true)
  {
    while (i2cRead(HMC5883L, 0x03, i2cData, 6));
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);
  }
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();



  //updatePitchRoll();


  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s

  gyroRate(gyroXrate, gyroYrate, gyroZrate);


  if ((pitch < -90 && kalPitch > 90) || (pitch > 90 && kalPitch < -90)) {
    kalmanY.setAngle(pitch);
    kalPitch = pitch;
    gyroYangle = pitch;
  }
  else
    kalPitch = kalmanY.getAngle(pitch, gyroPitch, dt);

  if (abs(kalPitch) > 90)
  {
    //gyroXrate = -gyroXrate;
    kalRoll = kalmanX.getAngle(roll, -gyroRoll, dt); // Calculate the angle using a Kalman filter
    gyroXangle -= gyroRoll * dt;
  }
  else
  {
    kalRoll = kalmanX.getAngle(roll, gyroRoll, dt);
    gyroXangle += gyroRoll * dt;
  }


  /* Yaw estimation */
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  magX *= magGain[0];
  magY *= magGain[1];
  magZ *= magGain[2];

  magX -= magOffset[0];
  magY -= magOffset[1];
  magZ -= magOffset[2];

  double rollAngle = kalRoll * DEG_TO_RAD;
  double pitchAngle = kalPitch * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

  yaw *= -1;

  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalYaw > 90) || (yaw > 90 && kalYaw < -90)) {
    kalmanZ.setAngle(yaw);
    kalYaw = yaw;
    gyroZangle = yaw;
  }
  else
    kalYaw = kalmanZ.getAngle(yaw, gyroYaw, dt); // Calculate the angle using a Kalman filter


  /* Estimate angles using gyro only */
  // Calculate gyro angle without any filter
  gyroYangle += gyroPitch * dt;
  gyroZangle += gyroYaw * dt;


  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalRoll;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalPitch;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalYaw;
    if(hmc==false)
    {
      kalYaw=0.00;
    }
}

void setAttitudeAndRate()
{
  //this function calculates setpoint of angular velocity and angles like roll, pitch and yaw
  if (!zero_ref)
  {
    //zero_ref variables gets true when it succesfully calculates setpoint 
    //time frame from 4sec to 7 sec from the start of arduino is chosen because only in this frame 
    //sensors output gets stable 
    if (millis() - timer_sp > 4000 && !(millis() - timer_sp > 7000))
    {
      //all these equation are averaging equation ,i,e, it averages output from sensors in the time frame from 4sec to 7 sec from the start
      //of arduino
        //after this time frame function is not executed 
      zero_angle_roll = compAngleSet * zero_angle_roll + (1 - compAngleSet) * kalRoll;
      zero_angle_pitch = compAngleSet * zero_angle_pitch + (1 - compAngleSet) * kalPitch;
      zero_angle_yaw = compAngleSet * zero_angle_yaw + (1 - compAngleSet) * kalYaw;

      zero_rate_roll = compRateSet * zero_rate_roll + (1 - compRateSet) * gyroRoll;
      zero_rate_pitch = compRateSet * zero_rate_pitch + (1 - compRateSet) * gyroPitch;
      zero_rate_yaw = compRateSet * zero_rate_yaw + (1 - compRateSet) * gyroYaw;

    }

    else if (millis() - timer_sp > 7000)
    {
      zero_ref = true;
      led_status(LOW, HIGH, LOW);

      //hanshake is done after 7seconds ,i,e, after setpoint are calculated
      //during setpoint time frame no motor is driven or started
      //syn_abc(); function is used for handshaking
      //to disable the handhake procedure just change the global variable HANDSHAKE
      //DON"T COMMENT OUT THIS FUNCTION(syn_sbc())
      syn_sbc();

      //it activiates the PID functions
      //it is used here as before 7sec, motors are not being driven and handshake is in while loop until handhskaee procesdure gets finishe

      //time taked in handshaking depends upon response from gui
      //arduino is ready for hanshaking after it has calculated setpoints
      PIDrol_rate.reset();
      PIDpitch_rate.reset();
      PIDyaw_rate.reset();

      PIDrol_angle.reset();
      PIDpitch_angle.reset();
      PIDyaw_angle.reset();
    }
  }
}

void gyroRate(double gyroXrate, double gyroYrate, double gyroZrate)
{
  //again it is averaging function used to remove noise
  //it is executed in each iteration of loop()
  gyroRoll = (compGyroFilter * gyroRoll + (1 - compGyroFilter) * gyroXrate);
  gyroPitch = (compGyroFilter * gyroPitch + (1 - compGyroFilter) * gyroYrate);
  gyroYaw = (compGyroFilter * gyroYaw + (1 - compGyroFilter) * gyroZrate);
}



