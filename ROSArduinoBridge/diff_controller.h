
#include"motor_driver.h"/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftrearPID,leftfrontPID ,rightrearPID, rightfrontPID;

/* PID Parameters */
float Kp = 0.01;
float Kd = 0.04;
float Ki = 0.0;
float Ko = 1.0;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftrearPID.TargetTicksPerFrame = 0.0;
   leftrearPID.Encoder = readEncoder(LEFT_REAR);
   leftrearPID.PrevEnc = leftrearPID.Encoder;
   leftrearPID.output = 0;
   leftrearPID.PrevInput = 0;
   leftrearPID.ITerm = 0;

   leftfrontPID.TargetTicksPerFrame = 0.0;
   leftfrontPID.Encoder = readEncoder(LEFT_FRONT);
   leftfrontPID.PrevEnc = leftfrontPID.Encoder;
   leftfrontPID.output = 0;
   leftfrontPID.PrevInput = 0;
   leftfrontPID.ITerm = 0;

   rightrearPID.TargetTicksPerFrame = 0.0;
   rightrearPID.Encoder = readEncoder(RIGHT_REAR);
   rightrearPID.PrevEnc = rightrearPID.Encoder;
   rightrearPID.output = 0;
   rightrearPID.PrevInput = 0;
   rightrearPID.ITerm = 0;

   rightfrontPID.TargetTicksPerFrame = 0.0;
   rightfrontPID.Encoder = readEncoder(RIGHT_FRONT);
   rightfrontPID.PrevEnc = rightfrontPID.Encoder;
   rightfrontPID.output = 0;
   rightfrontPID.PrevInput = 0;
   rightfrontPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftrearPID.Encoder = readEncoder(LEFT_REAR);
  leftfrontPID.Encoder = readEncoder(LEFT_FRONT);
  rightrearPID.Encoder = readEncoder(RIGHT_REAR);
  rightfrontPID.Encoder = readEncoder(RIGHT_FRONT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftrearPID.PrevInput != 0 || rightrearPID.PrevInput != 0 || leftfrontPID.PrevInput != 0 || rightfrontPID.PrevInput != 0 ) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightrearPID);
  doPID(&leftrearPID);
  doPID(&rightfrontPID);
  doPID(&leftfrontPID);

  /* Set the motor speeds accordingly */
//  setMotorSpeeds(leftPID.output, rightPID.output);
    setMotorSpeeds(leftrearPID.output, leftfrontPID.output , rightrearPID.output , rightfrontPID.output);
}

