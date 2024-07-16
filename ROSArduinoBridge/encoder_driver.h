/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_REAR_ENC_PIN_A PB7  //pin 13
  #define LEFT_REAR_ENC_PIN_B PB6  //pin 12
  #define LEFT_FRONT_ENC_PIN_A PB5 //pin 11
  #define LEFT_FRONT_ENC_PIN_B PB4  //pin 10
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_FRONT_ENC_PIN_A PK0 //pin A8
  #define RIGHT_FRONT_ENC_PIN_B PK1   //pin A9
  #define RIGHT_REAR_ENC_PIN_A PK2  //pin A10
  #define RIGHT_REAR_ENC_PIN_B PK3   //pin A11

#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

