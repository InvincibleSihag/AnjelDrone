int MFRvalue,MFLvalue,MBRvalue,MBLvalue;
float rollIntegeral,rollDerivative,pitchIntegeral,pitchDerivative,lastRollError,lastPitchError;
float rollKp=1,pitchKp=1,Ki=1.3,pitchKd=2,rollKd=2;
float rollMotor=0,pitchMotor=0,yawMotor=0;
void goUpDown()
{ 
  //Serial.println(value);
  MFR.writeMicroseconds(MFRvalue);
  MFL.writeMicroseconds(MFLvalue);
  MBR.writeMicroseconds(MBRvalue);
  MBL.writeMicroseconds(MBLvalue);
  /*Serial.print(MFLvalue);Serial.print("         ");Serial.print(MFRvalue);
  Serial.print("\n");
  Serial.print(MBLvalue);Serial.print("         ");Serial.print(MBRvalue);
  Serial.print("\n");*/
}

void MotorMixing()
{ 
  
  MFRvalue = thrust+yawMotor+pitchMotor+rollMotor;
  MFLvalue = thrust-yawMotor+pitchMotor-rollMotor;
  MBRvalue = thrust-yawMotor-pitchMotor+rollMotor;
  MBLvalue = thrust+yawMotor-pitchMotor-rollMotor;
  //Serial.println(MFLvalue);
  goUpDown();
}

void PIDroll(int elapsedTime,float error)
{
  rollDerivative = ((error-lastRollError)/elapsedTime)*100;
  lastRollError = error;
  rollMotor = -(rollKp*error + rollKd*rollDerivative);
  //Serial.println(rollDerivative*1000);
  MotorMixing();
}

void PIDpitch(int elapsedTime,float error)
{ 
  pitchDerivative = ((error-lastPitchError)/elapsedTime)*100;
  lastPitchError = error;
  //Serial.println(pitchDerivative*100);
  pitchMotor = (pitchKp*error + pitchKd*pitchDerivative);
  //Serial.println(String(pitchMotor)+"\t"+String(rollMotor));
  MotorMixing();
}

void PIDthrust()
{
  
}
