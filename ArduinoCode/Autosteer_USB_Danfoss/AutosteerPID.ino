void calcSteeringPID(void) 
 {
  
  //Proportional
  pidOutput = steerSettings.Kp * steerAngleError ; 
  //Integral
  outputSum += (steerSettings.Ki * steerAngleError);
  if (outputSum < -steerSettings.maxIntegralValue){
    outputSum = -steerSettings.maxIntegralValue;
  }
  else if (outputSum > steerSettings.maxIntegralValue){
    outputSum = steerSettings.maxIntegralValue;
  }
  //Derivative
  dInput = steerAngleError - lastSteerAngleError;
  pidOutput += outputSum - (steerSettings.Kd * dInput); 
  
  pwmDrive = (constrain(pidOutput, -250, 250));

  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0 ) pwmDrive -= steerSettings.minPWMValue;
  else if (pwmDrive > 0 ) pwmDrive += steerSettings.minPWMValue;
  
  if (pwmDrive > 255) pwmDrive = 255;
  if (pwmDrive < -255) pwmDrive = -255;

  if (aogSettings.MotorDriveDirection) pwmDrive *= -1;

  lastSteerAngleError = steerAngleError;
  
 }

//#########################################################################################


void motorDrive(void)
{
  // Used with Cytron MD30C Driver
  // Steering Motor
  // Dir + PWM Signal
  if (aogSettings.CytronDriver)
  {
    pwmDisplay = pwmDrive;

    //fast set the direction accordingly (this is pin DIR1_RL_ENABLE, port D, 4)
    if (pwmDrive >= 0) bitSet(PORTD, 4);  //set the correct direction
    else
    {
      bitClear(PORTD, 4);
      pwmDrive = -1 * pwmDrive;
    }

    //write out the 0 to 255 value
    analogWrite(PWM1_LPWM, pwmDrive);
  }
  else
  { // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    // Used with IBT 2  Driver for Danfoss
    // Dir1 connected to BOTH enables
    // PWM Left + PWM Right Signal

    pwmDrive = pwmDrive / 4;
    pwmDrive = pwmDrive + 128; // add Center Pos.

    pwmDisplay = pwmDrive;
    analogWrite(PWM1_LPWM, pwmDrive); // apply varying voltage to the VU valve pin
  }
}
