/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#ifdef BTS_MOTOR_DRIVER

 //RIGHT REARSIDE MOTORS
  #define R_PWM_RIGHT_SIDE_REAR 2 //PWM 2
  #define L_PWM_RIGHT_SIDE_REAR 3 //PWM 3
  #define L_EN_RIGHT_SIDE_REAR  23 //23
  #define R_EN_RIGHT_SIDE_REAR  25 //25
  
//LEFT REAR SIDE MOTORS
  #define R_PWM_LEFT_SIDE_REAR   4 //PWM 4
  #define L_PWM_LEFT_SIDE_REAR   5 //PWM 5
  #define L_EN_LEFT_SIDE_REAR    27 //27
  #define R_EN_LEFT_SIDE_REAR    29 //29

   //RIGHT FRONT SIDE MOTORS
  #define R_PWM_RIGHT_SIDE_FRONT 6 //PWM 6
  #define L_PWM_RIGHT_SIDE_FRONT 7 //PWM 7
  #define L_EN_RIGHT_SIDE_FRONT  53 //53
  #define R_EN_RIGHT_SIDE_FRONT  51 //51
  
//LEFT FRONT SIDE MOTORS
  #define R_PWM_LEFT_SIDE_FRONT   8 //PWM 8
  #define L_PWM_LEFT_SIDE_FRONT   9 //PWM 9
  #define L_EN_LEFT_SIDE_FRONT    50 //50
  #define R_EN_LEFT_SIDE_FRONT    52 //52


  
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftrearSpeed, int rightrearSpeed, int leftfrontSpeed, int rightfrontSpeed);

