void debug_terminal() {
  /* Print Data */

#if debug
#if magnet
  Serial.print(magX);
  Serial.print("\t");
  Serial.print(magY);
  Serial.print("\t");
  Serial.print(magZ);
  Serial.print("\t");
#endif

#if temp
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature);
  Serial.print("\t");
#endif

#if file_debug
  Serial.print(yawError);
  Serial.print("\t");
  Serial.print(altError);
  Serial.print("\t");
#endif



#if gyro
  //Serial.print(gyroXrate);
  // Serial.print("\t");
  Serial.print(gyroRoll);
  Serial.print("\t");
  //Serial.print(gyroYrate);
  //Serial.print("\t");
  Serial.print(gyroPitch);
  Serial.print("\t");
  //Serial.print(gyroZrate);
  //Serial.print("\t");
  Serial.print(gyroYaw);
  Serial.print("\t");
#endif


#if acc
  Serial.print(accX / 16384.0);
  Serial.print("\t");
  Serial.print(accY / 16384.0);
  Serial.print("\t");
  Serial.print(accZ / 16384.0);
  Serial.print("\t");
#endif


#if angle
  Serial.print(kalRoll);
  Serial.print("\t");
  Serial.print(kalPitch);
  Serial.print("\t");
  Serial.print(kalYaw);
  Serial.print("\t");
#endif

#if pid
  Serial.print(out_rate_x);
  Serial.print("\t");
  Serial.print(out_rate_y);
  Serial.print("\t");
  Serial.print(out_rate_z);
  Serial.print("\t");
  Serial.print(out_angle_x);
  Serial.print("\t");
  Serial.print(out_angle_y);
  Serial.print("\t");
  Serial.print(out_angle_z);
  Serial.print("\t");
  Serial.print(out_alt);
  Serial.print("\t");
#endif

#if zero_reference
  Serial.print(zero_angle_roll);
  Serial.print("\t");
  Serial.print(zero_angle_pitch);
  Serial.print("\t");
  Serial.print(zero_angle_yaw);
  Serial.print("\t");
  Serial.print(zero_rate_roll);
  Serial.print("\t");
  Serial.print(zero_rate_pitch);
  Serial.print("\t");
  Serial.print(zero_rate_yaw);
  Serial.print("\t");

#endif



#if (zero_reference) || (magnet) ||  (pid) ||  (gyro) || (acc) ||  (angle) || (temp)// not able to understand
  Serial.print("\r\n");
#endif

#endif
}

