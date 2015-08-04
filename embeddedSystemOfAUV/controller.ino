void controller() {

  if (handshake == true && zero_ref == true)
  {
    if (pid_angle == true)
    {
      out_angle_x = PIDrol_angle.ComputeFixedHz(kalRoll, zero_angle_roll, 20);
      out_angle_y = PIDpitch_angle.ComputeFixedHz(kalPitch, zero_angle_pitch, 20);
      //out_angle_z = PIDyaw_angle.ComputeFixedHz(kalYaw, zero_angle_yaw, 20);
      out_angle_z = PIDyaw_angle.ComputeFixedHz(yawError, 0, 20);

    }

    if (pid_rate_angle == false)

    {
      out_rate_x = PIDrol_rate.Compute(gyroRoll, zero_rate_roll);
      out_rate_y = PIDpitch_rate.Compute(gyroPitch, zero_rate_pitch);
      out_rate_z = PIDyaw_rate.Compute(gyroYaw, zero_rate_yaw);
    }
    else if (pid_angle == true)
    {
      out_rate_x = PIDrol_rate.Compute(gyroRoll, zero_rate_roll, out_angle_x);
      out_rate_y = PIDpitch_rate.Compute(gyroPitch, zero_rate_pitch, out_angle_y);
      out_rate_z = PIDyaw_rate.Compute(gyroYaw, zero_rate_yaw, out_angle_z);
    }
    
    if(pid_alt==true)
    {
      out_alt=PIDalt.Compute(altError,0,0);
      b_speed=constrain(threshold+int(out_alt),0,200);
    }

    analogWrite(l_pwm, constrain(f_speed + int(out_rate_z), 0, maximun_pwm));
    analogWrite(r_pwm, constrain(f_speed - int(out_rate_z), 0, maximun_pwm));

    analogWrite(lf_pwm, constrain(b_speed - int(out_rate_x) - int(out_rate_y), 0, maximun_pwm));
    analogWrite(rf_pwm, constrain(b_speed + int(out_rate_x) - int(out_rate_y), 0, maximun_pwm));

    analogWrite(lb_pwm, constrain(b_speed - int(out_rate_x) + int(out_rate_y), 0, maximun_pwm));
    analogWrite(rb_pwm, constrain(b_speed + int(out_rate_x) + int(out_rate_y), 0, maximun_pwm));


#if debug
    debug_terminal();
#endif
  }
}

void motorArm() {
  //motors
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(lf_pwm, OUTPUT);
  pinMode(rf_pwm, OUTPUT);
  pinMode(lb_pwm, OUTPUT);
  pinMode(rb_pwm, OUTPUT);

  pinMode(l_dir, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(lf_dir, OUTPUT);
  pinMode(rf_dir, OUTPUT);
  pinMode(lb_dir, OUTPUT);
  pinMode(rb_dir, OUTPUT);

  digitalWrite(l_dir, LOW);
  digitalWrite(r_dir, LOW);
  digitalWrite(lf_dir, LOW);
  digitalWrite(rf_dir, LOW);
  digitalWrite(lb_dir, LOW);
  digitalWrite(rb_dir, LOW);


}


