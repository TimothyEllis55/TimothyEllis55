#include <FlashStorage.h>
#include <AccelStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, 10, 9); // (10 = step) ; (9 = dir)
AccelStepper stepper2(AccelStepper::DRIVER, 21, 20); // (21 = step) ; (20 = dir)

#include <RotaryEncoder.h>

//#define PIN_SWITCH

#define enc1_dt                          13
#define enc1_clk                         12
#define enc1_sw                           5 // was on pin 11 (interrupt 0) caused crashing

#define enc2_dt                          23
#define enc2_clk                         22
#define enc2_sw                          24

RotaryEncoder encoder1(enc1_dt, enc1_clk, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder2(enc2_dt, enc2_clk, RotaryEncoder::LatchMode::TWO03);

#define step_pin                         10
#define dir_pin                           9

#define CCW                               0
#define CW                                1

#define MAX_SPEED                       2000 // Stepper speed in steps per second
#define ACCELERATION                   10000 // Stepper acceleration in (steps per second)^2
#define RUNNING                           1
#define STOPPED                           0

#define NO_INIT                           0
#define INIT                              1

#define FALSE                             0
#define TRUE                              1

#define MOTOR_STEPS_PER_ROTATION        200
#define GEAR_RATIO                      100
#define STEP_FACTOR                       1
#define GEARED_STEPS_PER_ROTATION       (MOTOR_STEPS_PER_ROTATION * GEAR_RATIO) // 20000

#define HOME_SCREEN_INIT                  0
#define HOME_SCREEN                       1
#define DATA_INPUT_SCREEN_INIT            2
#define DATA_INPUT_SCREEN                 3

#define BUTTON_ID_03                      3
#define BUTTON_ID_04                      4
#define BUTTON_ID_06                      6
#define BUTTON_ID_07                      7

#define DISPLAY_RATE_MS                 100 // period to update display in ms

int analogPin_1 = A0;               // Hall effect sensor on A0
int analogPin_2 = A1;               // Hall effect sensor on A1
int analogVal_1 = 0;
int analogVal_2 = 0;

int screenState = HOME_SCREEN_INIT; // start at home screen init state

volatile int encoderValue = 0;
volatile int turnDetected = 0;
volatile int pinSwitchDetected = 0;

int display_period = DISPLAY_RATE_MS;
unsigned long time_now_display = 0;
unsigned long time_now_adc = 0;
unsigned long time_now_stepper = 0;
int adc_period = 100;
int stepper_period = 1;

int Bx;
int By;

int Stepper_1_Position = 0;
int Stepper_2_Position = 0;

int PinSwitchFlag = 0;
int SavedEncoder_1_Value = 0;
int SavedEncoder_2_Value = 0;
int motorDirectionChange = FALSE;
int Encoder_Index = 0;

typedef struct
{
  int nvm_motor_1_position;
  int nvm_motor_2_position;
  int nvm_encoder_1_value;
  int nvm_encoder_2_value;
  int nvm_saved_encoder_1_value;
  int nvm_saved_encoder_2_value;
  int nvm_Bx;
  int nvm_By;
}storage;

FlashStorage(flash_store, storage);
storage flash_variables;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  //pinMode(enc_dt, INPUT_PULLUP);  
  //pinMode(enc_clk, INPUT_PULLUP);
  pinMode(enc1_sw, INPUT_PULLUP);
  pinMode(enc2_sw, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(enc_clk), encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc1_sw), pin_switch, FALLING);

  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setSpeed(4000);
  stepper1.setAcceleration(ACCELERATION);
  
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setSpeed(4000);
  stepper2.setAcceleration(ACCELERATION);

  restoreNVM(); // read nvm and restore stored variables
}

void loop()
{
  switch(screenState)
  {
    case HOME_SCREEN_INIT:
      encoder1.setPosition(SavedEncoder_1_Value);
      //encoder2.setPosition(SavedEncoder_2_Value);
      updateDisplay(Stepper_1_Position, Stepper_2_Position, INIT);          // Update display at initialisation
      screenState = HOME_SCREEN;
      break;
    
    case HOME_SCREEN:
      encoder1.tick();
      encoder2.tick();
      Encoder_Index = readRotaryEncoder();                           // Read rotary encoder for distance and direction
      AccelStepper_run(Stepper_1_Position, Stepper_2_Position, Encoder_Index);
      updateDisplay(Stepper_1_Position, Stepper_2_Position, NO_INIT);       // Update display before move to new position
      readHallEffectSensors();
      displayRun();
      break;

    case DATA_INPUT_SCREEN_INIT:                     // state to enter once before main state
      data_input_screen_run(INIT);
      screenState = DATA_INPUT_SCREEN;
      break;
    
    case DATA_INPUT_SCREEN:
      data_input_screen_run(NO_INIT);
      encoder1.tick();
      displayRun();
      break;
  }
}

int readRotaryEncoder()
{
  int RotaryValue_1 = 0;
  int RotaryValue_2 = 0;
  static int encoder_index = 0;

  if((int)encoder1.getDirection() != 0)
  {
    RotaryValue_1 = (encoder1.getPosition() / 2) * GEAR_RATIO * STEP_FACTOR;

    if(RotaryValue_1 >= GEARED_STEPS_PER_ROTATION)
    {
      RotaryValue_1 = GEARED_STEPS_PER_ROTATION;
      encoder1.setPosition((MOTOR_STEPS_PER_ROTATION / STEP_FACTOR) * 2); // x2 because encoder reports two steps per indent
    }
    else if(RotaryValue_1 < 0)
    {
      RotaryValue_1 = 0;
      encoder1.setPosition(0);
    }
    else
    {
      //do nothing
    }
    Stepper_1_Position = RotaryValue_1;
    SavedEncoder_1_Value = encoder1.getPosition();
    encoder_index = 1;
  }
  else if((int)encoder2.getDirection() != 0)
  {
    RotaryValue_2 = (encoder2.getPosition() / 2) * GEAR_RATIO * STEP_FACTOR;

    if(RotaryValue_2 >= GEARED_STEPS_PER_ROTATION)
    {
      RotaryValue_2 = GEARED_STEPS_PER_ROTATION;
      encoder2.setPosition((MOTOR_STEPS_PER_ROTATION / STEP_FACTOR) * 2); // x2 because encoder reports two steps per indent
    }
    else if(RotaryValue_2 < 0)
    {
      RotaryValue_2 = 0;
      encoder2.setPosition(0);
    }
    else
    {
      //do nothing
    }
    Stepper_2_Position = RotaryValue_2;
    SavedEncoder_2_Value = encoder2.getPosition();
    encoder_index = 2;
  }
  return encoder_index;
}

void AccelStepper_run(int stepper_1_Pos, int stepper_2_Pos, int encoder_index)
{
  //int stepper_1_pos = stepper_1_Pos;
  //int stepper_2_pos = stepper_2_Pos;
  int stepper_1_pos = 0;
  int stepper_2_pos = 0;
  static int motorDirectionChangePrev = 0;
  static int encoder_index_prev = 0;
  
  //Serial.println(stepper_1_pos);
  //Serial.println(stepper_2_pos);
  
#ifdef PIN_SWITCH 
  if(pinSwitchDetected == 1)
  {
    pinSwitchDetected = 0;
#endif

    if(encoder_index == 2 && encoder_index_prev == 1)
    {
      stepper2.setCurrentPosition(-stepper2.currentPosition());
      encoder_index_prev = encoder_index;
    }
    else if(encoder_index == 1 && encoder_index_prev == 2)
    {
      stepper2.setCurrentPosition(-stepper2.currentPosition());
      encoder_index_prev = encoder_index;
    }
    if(encoder_index == 1)
    {
      stepper_1_pos = stepper_1_Pos;
      stepper_2_pos = stepper_1_Pos;
    }
    else if(encoder_index == 2)
    {
      stepper_1_pos = stepper_1_Pos;
      stepper_2_pos = -stepper_1_Pos;
    }
    //Serial.println(encoder_index);
    encoder_index_prev = encoder_index;
//    if(motorDirectionChange == TRUE && motorDirectionChangePrev == FALSE)
//    {
//      stepper2.setCurrentPosition(-stepper2.currentPosition());
//      motorDirectionChangePrev = motorDirectionChange;
//    }
//    else if(motorDirectionChange == FALSE && motorDirectionChangePrev == TRUE)
//    {
//      stepper2.setCurrentPosition(-stepper2.currentPosition());
//      motorDirectionChangePrev = motorDirectionChange;
//    }
//    if(motorDirectionChange == TRUE)
//    {
//      stepper_2_pos = -stepper_2_Pos;  // run stepper 2 in opposite direction to stepper 1
//    }
//    else if(motorDirectionChange == FALSE)
//    {
//      stepper_2_pos = stepper_2_Pos;  // run stepper 2 in opposite direction to stepper 1
//    }
    
      stepper1.moveTo(stepper_1_pos);
      stepper2.moveTo(stepper_2_pos);


#ifdef PIN_SWITCH
    displayMotorStatus(RUNNING);
  }
#endif
  stepper1.run();
  stepper2.run();

#ifdef PIN_SWITCH 
  if(stepper1.distanceToGo() == 0)
  {
    displayMotorStatus(STOPPED);
  }
#endif
}

//void encoder()
//{
//  turnDetected = 1;
//  (digitalRead(enc_clk) == digitalRead(enc_dt)) ? (encoderValue--) : (encoderValue++);
//}

void updateDisplay(int stepper_1_pos, int stepper_2_pos, int init)
{
  static int stepper_1_currentValue = 0;
  static int stepper_2_currentValue = 0;
  int stepper_1_displayValue = 0;
  int stepper_2_displayValue = 0;
  //displayValue = (stepper_pos * 360 / 20000);
  //displayValue = (311*cos((2*PI*stepper_pos)/20000));

  if((millis() >= time_now_display + display_period) || (init == 1))
  {
    time_now_display += display_period;
    stepper_1_displayValue = stepper_1_pos;
    stepper_2_displayValue = stepper_2_pos;

    if((stepper_1_currentValue != stepper_1_displayValue) || (init == 1))
    {
      if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
      {
        stepper_1_displayValue = (311 * cos((2 * PI * stepper_1_pos)/20000));
        Serial1.print("n0.val=");
        Serial1.print(stepper_1_displayValue);
        Serial1.print("\xFF\xFF\xFF");
        stepper_1_displayValue = stepper_1_pos;  
        stepper_1_currentValue = stepper_1_displayValue;
      }
    }
    if((stepper_2_currentValue != stepper_2_displayValue) || (init == 1))
    {
      if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
      {
        stepper_2_displayValue = (311 * cos((2 * PI * stepper_2_pos)/20000));
        Serial1.print("n3.val=");
        Serial1.print(stepper_2_displayValue);
        Serial1.print("\xFF\xFF\xFF");
        stepper_2_displayValue = stepper_2_pos;  
        stepper_2_currentValue = stepper_2_displayValue;
      }
    }
  }
}

void displayRun()
{
  byte received_data[7];
  //for(int i = 0; i < 7; i++)
  //{
  //  received_data[i] = 0;
  //}
  while(Serial1.available() > 0)
  {
    Serial1.readBytesUntil('\n', received_data, 7);
    byte buttonID = received_data[2];            // position 2 has the button ID
    
    switch(buttonID)
    {
      case BUTTON_ID_03:
        Serial.println("Store was pressed!");
        saveToFlash();
        break;

      case BUTTON_ID_04:
        screenState = DATA_INPUT_SCREEN_INIT;    // button pg2 has been pressed - this is data entry screen
        break;

      case BUTTON_ID_06:
        (motorDirectionChange) ? (motorDirectionChange = FALSE) : (motorDirectionChange = TRUE);
        (motorDirectionChange) ? (Serial1.print("b2.txt=\"OPP\"\xFF\xFF\xFF")) : (Serial1.print("b2.txt=\"SAME\"\xFF\xFF\xFF"));
        break;

      case BUTTON_ID_07:
        screenState = HOME_SCREEN_INIT;
        break;
    }
  }
}

void pin_switch()
{
  pinSwitchDetected = 1;
}

void restoreNVM()
{
  flash_variables = flash_store.read();                             // read structure
  Stepper_1_Position = flash_variables.nvm_motor_1_position;        // restore stepper 1 position
  Stepper_2_Position = flash_variables.nvm_motor_2_position;        // restore stepper 2 position
  encoder1.setPosition(flash_variables.nvm_encoder_1_value);       // restore encoder 1 value
  encoder2.setPosition(flash_variables.nvm_encoder_2_value);       // restore encoder 2 value
  SavedEncoder_1_Value = flash_variables.nvm_saved_encoder_1_value; // restore saved encoder 1 value
  SavedEncoder_2_Value = flash_variables.nvm_saved_encoder_2_value; // restore saved encoder 2 value
  Bx = flash_variables.nvm_Bx;                                      // restore Bx
  By = flash_variables.nvm_By;                                      // restore By
}

void saveToFlash()
{
  flash_variables.nvm_motor_1_position = Stepper_1_Position;          // save stepper 1 position
  flash_variables.nvm_motor_2_position = Stepper_2_Position;          // save stepper 2 position
  flash_variables.nvm_encoder_1_value = encoder1.getPosition();     // save encoder 1 value
  flash_variables.nvm_encoder_2_value = encoder2.getPosition();     // save encoder 2 value
  flash_variables.nvm_saved_encoder_1_value = SavedEncoder_1_Value; // save backup of encoder 1 value
  flash_variables.nvm_saved_encoder_2_value = SavedEncoder_2_Value; // save backup of encoder 2 value
  flash_variables.nvm_Bx = Bx;                                      // save Bx
  flash_variables.nvm_By = By;                                      // save By
  flash_store.write(flash_variables);                               // save structure
}

void data_input_screen_run(int init)
{
  static int Bx_previous_value = 0;
  static int By_previous_value = 0;
  int encoder_snapshot = 0;
  int pinSwChange = 0;

  if(init)
  {
    Bx_previous_value = 0;
    By_previous_value = 0;
    //SavedEncoderValue = encoderValue;
    SavedEncoder_1_Value = encoder1.getPosition();
    //encoderValue = 0;
    encoder1.setPosition(0);
    setValueFocus(0);
    PinSwitchFlag = 0;

    Serial1.print("n0.val=");
    Serial1.print(Bx);
    Serial1.print("\xFF\xFF\xFF");

    Serial1.print("n1.val=");
    Serial1.print(By);
    Serial1.print("\xFF\xFF\xFF");
    return;
  }

  if(pinSwitchDetected)
  {
    pinSwitchDetected = 0;
    (PinSwitchFlag) ? (PinSwitchFlag = 0) : (PinSwitchFlag = 1);
    //encoderValue = 0;
    encoder1.setPosition(0);
    Bx_previous_value = 0;
    By_previous_value = 0;
    //turnDetected = 1;
    pinSwChange = 1;
  }

  //if(turnDetected)
  if(((int)encoder1.getDirection() != 0) || (pinSwChange == 1))
  {
    pinSwChange = 0;
    //turnDetected = 0;
    //encoder_snapshot = encoderValue;
    encoder_snapshot = (encoder1.getPosition() / 2);

    if(PinSwitchFlag == 0)
    {
      //Serial.println(encoderValue);
      //Serial.println(Bx_previous_value);
      //Serial.println(Bx);
      //encoder_snapshot = encoderValue;
      setValueFocus(0);
      Bx += (encoder_snapshot - Bx_previous_value);
      Bx_previous_value = encoder_snapshot;
      Serial1.print("n0.val=");
      Serial1.print(Bx);
      Serial1.print("\xFF\xFF\xFF");
    }
    else if(PinSwitchFlag == 1)
    {
      //Serial.println(encoderValue);
      //Serial.println(By_previous_value);
      //Serial.println(By);
      //encoder_snapshot = encoderValue;
      setValueFocus(1);
      By += (encoder_snapshot - By_previous_value);
      By_previous_value = encoder_snapshot;
      Serial1.print("n1.val=");
      Serial1.print(By);
      Serial1.print("\xFF\xFF\xFF");
    }
  }
}

void data_input_screen_init()
{
  //SavedEncoderValue = encoderValue;
  SavedEncoder_1_Value = encoder1.getPosition();
  
  //encoderValue = 0;
  encoder1.setPosition(0);
  setValueFocus(0);
  PinSwitchFlag = 0;

  Serial1.print("n0.val=");
  Serial1.print(Bx);
  Serial1.print("\xFF\xFF\xFF");

  Serial1.print("n1.val=");
  Serial1.print(By);
  Serial1.print("\xFF\xFF\xFF");
}

void setValueFocus(int item)
{
  if(item == 0)
  {
    Serial1.print("n0.bco=0");
    Serial1.print("\xFF\xFF\xFF");
    Serial1.print("n0.pco=65535");
    Serial1.print("\xFF\xFF\xFF");

    Serial1.print("n1.bco=65535");
    Serial1.print("\xFF\xFF\xFF");
    Serial1.print("n1.pco=0");
    Serial1.print("\xFF\xFF\xFF");
  }
  else if(item == 1)
  {
    Serial1.print("n1.bco=0");
    Serial1.print("\xFF\xFF\xFF");
    Serial1.print("n1.pco=65535");
    Serial1.print("\xFF\xFF\xFF");

    Serial1.print("n0.bco=65535");
    Serial1.print("\xFF\xFF\xFF");
    Serial1.print("n0.pco=0");
    Serial1.print("\xFF\xFF\xFF");
  }
}

void displayMotorStatus(int run_stop)
{
  static int previous_value = 0;
  
  if(run_stop == 0 && (previous_value != run_stop))
  {
    Serial1.print("t1.txt=\"\"");
    Serial1.print("\xFF\xFF\xFF");
  }
  else if(run_stop == 1 && (previous_value != run_stop))
  {
    Serial1.print("t1.txt=\"RUNNING...\"");
    Serial1.print("\xFF\xFF\xFF");
  }
  previous_value = run_stop;
}

void readHallEffectSensors()
{
  if(millis() >= (time_now_adc + adc_period))
  {
    time_now_adc += adc_period;
    
    if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
    {
      analogReadResolution(12);
      analogVal_1 = analogRead(analogPin_1);
      analogVal_2 = analogRead(analogPin_2);
      Serial1.print("n1.val=");
      Serial1.print((analogVal_1 - 1644) * 3300 / 4096); // display in mV (mT?)
      Serial1.print("\xFF\xFF\xFF");
      Serial1.print("n2.val=");
      Serial1.print((analogVal_2 - 1656) * 3300 / 4096); // display in mV (mT?)
      Serial1.print("\xFF\xFF\xFF"); 
    }
  }
}
