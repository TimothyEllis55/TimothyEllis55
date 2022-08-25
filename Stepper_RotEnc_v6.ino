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

#define NO_RESET                          0
#define RESET                             1

#define NO_REVERSE                        0
#define REVERSE                           1

#define FALSE                             0
#define TRUE                              1

#define MOTOR_STEPS_PER_ROTATION        200
#define GEAR_RATIO                      100
#define STEP_FACTOR                       1
#define ROTARY_GAIN_FINE                 50
#define ROTARY_GAIN_COARSE              100
#define GEARED_STEPS_PER_ROTATION       (MOTOR_STEPS_PER_ROTATION * GEAR_RATIO) // 20000

#define HOME_SCREEN_INIT                  0
#define HOME_SCREEN                       1
#define HALL_PROBES_SCREEN                2
#define DATA_INPUT_SCREEN_INIT            3
#define DATA_INPUT_SCREEN                 4

#define BUTTON_ID_01                      1
#define BUTTON_ID_02                      2
#define BUTTON_ID_03                      3
#define BUTTON_ID_04                      4
#define BUTTON_ID_05                      5
#define BUTTON_ID_06                      6
#define BUTTON_ID_07                      7

#define DISPLAY_RATE_MS                 100 // period to update display in ms

#define HALL_FILTER_ONE( v, c, n )      ((v - (v / c))+(n / c))

int analogPin_1 = A0;               // Hall effect sensor on A0
int analogPin_2 = A1;               // Hall effect sensor on A1
int analogVal_1 = 0;
int analogVal_2 = 0;

int screenState = HOME_SCREEN_INIT; // start at home screen init state

volatile int encoderValue = 0;
volatile int turnDetected = 0;
volatile int pinSwitch_1_Detected = 0;
volatile int pinSwitch_2_Detected = 0;

int display_period = DISPLAY_RATE_MS;
unsigned long time_now_display = 0;
unsigned long time_now_adc = 0;
unsigned long time_now_stepper = 0;
int adc_period = 200;
int stepper_period = 1;

int Rotary_gain = ROTARY_GAIN_COARSE;

int B_max = 0;
int B_min = 0;

int Stepper_1_Position = 0;
int Stepper_2_Position = 0;

int RotaryValue_1 = 0;
int RotaryValue_2 = 0;

int Display_RotaryValue_1 = 0;
int Display_RotaryValue_2 = 0;

int Bdisp = 0;
int Tdisp = 0;

int SavedEncoder_1_Value = 0;
int SavedEncoder_2_Value = 0;
int Encoder_Index = 0;

typedef struct
{
  int nvm_motor_1_position;
  int nvm_motor_2_position;
  int nvm_encoder_1_value;
  int nvm_encoder_2_value;
  int nvm_saved_encoder_1_value;
  int nvm_saved_encoder_2_value;
  int nvm_B_max;
  int nvm_B_min;
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
  attachInterrupt(digitalPinToInterrupt(enc1_sw), pin_switch_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc2_sw), pin_switch_2, FALLING);

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
      encoder2.setPosition(SavedEncoder_2_Value);
      Serial.println(encoder1.getPosition());
      Serial.println(encoder2.getPosition());
      updateDisplay_Mag_Angle(Display_RotaryValue_1, Display_RotaryValue_2, INIT);          // Update display at initialisation
      screenState = HOME_SCREEN;
      break;
    
    case HOME_SCREEN:
      encoder1.tick();
      encoder2.tick();
      if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)
      {
        Encoder_Index = readRotaryEncoder();                           // Read rotary encoder for distance and direction
      }
      Check_rotary_gain();
      AccelStepper_run(Stepper_1_Position, Stepper_2_Position, Encoder_Index, NO_RESET);
      updateDisplay_Mag_Angle(Display_RotaryValue_1, Display_RotaryValue_2, NO_INIT);       // Update display before move to new position
      displayRun();
      break;

    case HALL_PROBES_SCREEN:
      encoder1.tick();
      encoder2.tick();
      if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)
      {
        Encoder_Index = readRotaryEncoder();                           // Read rotary encoder for distance and direction
      }
      AccelStepper_run(Stepper_1_Position, Stepper_2_Position, Encoder_Index, NO_RESET);
      hall_probes_screen_run();
      displayRun();
      break;

    case DATA_INPUT_SCREEN_INIT:                     // state to enter once before main state
      data_input_screen_run(INIT);
      screenState = DATA_INPUT_SCREEN;
      break;
    
    case DATA_INPUT_SCREEN:
      encoder1.tick();
      encoder2.tick();
      data_input_screen_run(NO_INIT);
      displayRun();
      break;
  }
}

int readRotaryEncoder()
{
  static int encoder_index = 0;

  if((int)encoder1.getDirection() != 0)
  {
    //RotaryValue_1 = ((encoder1.getPosition() / 2) * ROTARY_GAIN_COARSE * STEP_FACTOR);
    RotaryValue_1 = (encoder1.getPosition() / 2);
    Display_RotaryValue_1 = RotaryValue_1 * Rotary_gain;

    if(Display_RotaryValue_1 < 0)
    {
      Display_RotaryValue_1 += GEARED_STEPS_PER_ROTATION;
    }
    Stepper_1_Position = RotaryValue_1;
    SavedEncoder_1_Value = encoder1.getPosition();
    encoder_index = 1;
  }
  else if((int)encoder2.getDirection() != 0)
  {
    //RotaryValue_2 = (encoder2.getPosition() / 2) * ROTARY_GAIN_COARSE * STEP_FACTOR;
    RotaryValue_2 = (encoder2.getPosition() / 2);
    Display_RotaryValue_2 = RotaryValue_2 * Rotary_gain;

    if(Display_RotaryValue_2 < 0)
    {
      Display_RotaryValue_2 += GEARED_STEPS_PER_ROTATION;
    }
    Stepper_2_Position = RotaryValue_2;
    SavedEncoder_2_Value = encoder2.getPosition();
    encoder_index = 2;
  }
  return encoder_index;
}

void Check_rotary_gain()
{
  if(pinSwitch_1_Detected)
  {
    pinSwitch_1_Detected = 0;
    Rotary_gain = (Rotary_gain == ROTARY_GAIN_COARSE) ? ROTARY_GAIN_FINE : ROTARY_GAIN_COARSE;
  }
}

void AccelStepper_run(int stepper_1_Pos, int stepper_2_Pos, int encoder_index, int reset)
{
  static int stepper_1_Pos_prev = 0;
  static int stepper_2_Pos_prev = 0;
  int steps_to_move_1 = 0;
  int steps_to_move_2 = 0;

  //stepper_1_Pos = stepper_1_Pos * Rotary_gain;
  //stepper_2_Pos = stepper_2_Pos * Rotary_gain;
  
  static int encoder_index_prev = 0;

  if(reset)
  {
    stepper_1_Pos_prev = 0;
    stepper_2_Pos_prev = 0;
    return;
  }

  if((stepper_1_Pos != stepper_1_Pos_prev) && (encoder_index == 1)) // counter-rotation (Amplitude dial change)
  {
    steps_to_move_1 = (stepper_1_Pos - stepper_1_Pos_prev) * Rotary_gain; // get delta needed to move
    stepper_1_Pos_prev = stepper_1_Pos;
    steps_to_move_2 = -steps_to_move_1; // counter-rotation
    if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)
    {
      stepper1.setSpeed(4000);
      stepper1.move(steps_to_move_1);
      stepper2.setSpeed(4000);
      stepper2.move(steps_to_move_2);
//      Serial.println(RotaryValue_1);
//      Serial.println(stepper1.currentPosition());
    }
  }
  else if((stepper_2_Pos != stepper_2_Pos_prev) && (encoder_index == 2)) // co-rotation (Angle dial change)
  {
    steps_to_move_2 = (stepper_2_Pos - stepper_2_Pos_prev) * Rotary_gain; // get delta needed to move
    stepper_2_Pos_prev = stepper_2_Pos;
    steps_to_move_1 = steps_to_move_2; // co-rotation
    if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)
    {
      stepper1.setSpeed(4000);
      stepper1.move(steps_to_move_1);
      stepper2.setSpeed(4000);
      stepper2.move(steps_to_move_2);
//      Serial.println(RotaryValue_2);
//      Serial.println(stepper1.currentPosition());
    }
  }
 
  encoder_index_prev = encoder_index;
   
  stepper1.run();
  stepper2.run();
}

void updateDisplay_Mag_Angle(int Mag, int Angle, int init)
{
  static int Mag_prev = 0;
  static int Angle_prev = 0;

  int Angle_counts = 0;
  
  static int Bx_1 = 0;
  static int By_1 = 0;
  int B_disp = 0;
  int T_disp = 0;
  int Bx = 0;
  int By = 0;

  if((millis() >= time_now_display + display_period) || (init == 1))
  {
    //time_now_display += display_period;
    time_now_display = millis();

    if((Mag != Mag_prev) || Angle != Angle_prev || (init == 1))
    {
      if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
      {
        Mag_prev = Mag;
        Mag = (Mag % GEARED_STEPS_PER_ROTATION);
        By_1 = (B_max * cos((2 * PI * Mag) / GEARED_STEPS_PER_ROTATION));
        Bx_1 = (B_min * sin((2 * PI * Mag) / GEARED_STEPS_PER_ROTATION));

        B_disp = sqrt(sq(By_1) + sq(Bx_1));
        Serial1.print("n0.val=");
        Serial1.print(B_disp);
        Serial1.print("\xFF\xFF\xFF");
    //  }
    //}
    //if((Angle != Angle_prev) || (init == 1))
    //{
    //  if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
    //  {
        Angle_prev = Angle;
        Angle = (Angle % GEARED_STEPS_PER_ROTATION);
        Angle_counts = ((360 * Angle) / GEARED_STEPS_PER_ROTATION);

        //Serial.println("By_1");
        //Serial.println(By_1);
        //Serial.println("Bx_1");
        //Serial.println(Bx_1);
        //Serial.println("Angle Counts");
        //Serial.println(Angle_counts);

        T_disp = ((atan2(Bx_1, By_1) * 180 / PI));
        T_disp = (T_disp < 0) ? (T_disp + 360) : (T_disp);
        T_disp += Angle_counts;
        
//        
//        Serial.println("Angle_counts");
//        Serial.println(Angle_counts);
//        Serial.println("Bx_1");
//        Serial.println(Bx_1);
//        Serial.println("By_1");
//        Serial.println(By_1);
//        Serial.println("atan2(Bx_1, By_1);");
//        Serial.println(atan2(Bx_1, By_1) * 180 / PI);
//        Serial.println("T_disp");
//        Serial.println(T_disp % 360);
        
        Serial1.print("n1.val=");
        //Serial1.print(Angle_counts);
        Serial1.print(T_disp % 360);
        Serial1.print("\xFF\xFF\xFF");
     // }

        Bx = sin(T_disp*PI/180) * B_disp;
        By = cos(T_disp*PI/180) * B_disp;
        Serial1.print("n2.val=");
        Serial1.print(Bx);
        Serial1.print("\xFF\xFF\xFF");
        Serial1.print("n3.val=");
        Serial1.print(By);
        Serial1.print("\xFF\xFF\xFF");
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
      case BUTTON_ID_01:
        screenState = HOME_SCREEN_INIT;
        break;

      case BUTTON_ID_02:
        screenState = HALL_PROBES_SCREEN;        // button "Align" has been pressed - hall probes display
        break;

      case BUTTON_ID_03:
        screenState = DATA_INPUT_SCREEN_INIT;    // button "Calibrate" has been pressed - this is data entry screen
        break;

      case BUTTON_ID_04:                         // button "Set" has been pressed on Align screen
        Zero_All();                              // Zero all counters
        saveToFlash();
        break;

      case BUTTON_ID_06:                         // button "Set" has been pressed on Calibrate screen
        Serial.println("Store was pressed!");
        saveToFlash();
        break;
    }
  }
}

void pin_switch_1()
{
  pinSwitch_1_Detected = 1;
}

void pin_switch_2()
{
  pinSwitch_2_Detected = 1;
}

void restoreNVM()
{
  flash_variables = flash_store.read();                             // read structure
  Stepper_1_Position = flash_variables.nvm_motor_1_position;        // restore stepper 1 position
  Stepper_2_Position = flash_variables.nvm_motor_2_position;        // restore stepper 2 position
  encoder1.setPosition(flash_variables.nvm_encoder_1_value);        // restore encoder 1 value
  encoder2.setPosition(flash_variables.nvm_encoder_2_value);        // restore encoder 2 value
  SavedEncoder_1_Value = flash_variables.nvm_saved_encoder_1_value; // restore saved encoder 1 value
  SavedEncoder_2_Value = flash_variables.nvm_saved_encoder_2_value; // restore saved encoder 2 value
  B_max = flash_variables.nvm_B_max;                                // restore B_max
  B_min = flash_variables.nvm_B_min;                                // restore B_min
}

void saveToFlash()
{
  flash_variables.nvm_motor_1_position = Stepper_1_Position;        // save stepper 1 position
  flash_variables.nvm_motor_2_position = Stepper_2_Position;        // save stepper 2 position
  flash_variables.nvm_encoder_1_value = encoder1.getPosition();     // save encoder 1 value
  flash_variables.nvm_encoder_2_value = encoder2.getPosition();     // save encoder 2 value
  flash_variables.nvm_saved_encoder_1_value = SavedEncoder_1_Value; // save backup of encoder 1 value
  flash_variables.nvm_saved_encoder_2_value = SavedEncoder_2_Value; // save backup of encoder 2 value
  flash_variables.nvm_B_max = B_max;                                // save B_max
  flash_variables.nvm_B_min = B_min;                                // save B_min
  flash_store.write(flash_variables);                               // save structure
}

void data_input_screen_run(int init)
{
  static int B_max_prev = 0;
  static int B_min_prev = 0;
  int encoder_1_snapshot = 0;
  int encoder_2_snapshot = 0;
  static int encoder_1_scale = 1;
  static int encoder_2_scale = 1;

  if(init)
  {
    B_max_prev = 0;
    B_min_prev = 0;
    SavedEncoder_1_Value = encoder1.getPosition();
    SavedEncoder_2_Value = encoder2.getPosition();
    encoder1.setPosition(0);
    encoder2.setPosition(0);

    Serial1.print("n6.val=");
    Serial1.print(B_max);
    Serial1.print("\xFF\xFF\xFF");

    Serial1.print("n7.val=");
    Serial1.print(B_min);
    Serial1.print("\xFF\xFF\xFF");
    return;
  }
  if(pinSwitch_1_Detected)
  {
    pinSwitch_1_Detected = 0;
    encoder_1_scale = (encoder_1_scale == 1) ? (encoder_1_scale = 10) : (encoder_1_scale = 1);
  }
  if(pinSwitch_2_Detected)
  {
    pinSwitch_2_Detected = 0;
    encoder_2_scale = (encoder_2_scale == 1) ? (encoder_2_scale = 10) : (encoder_2_scale = 1);
  }
  if((int)encoder1.getDirection() != 0)
  {
    encoder_1_snapshot = (encoder1.getPosition() / 2);
    B_max += (encoder_1_snapshot - B_max_prev) * encoder_1_scale;
    B_max_prev = encoder_1_snapshot;
    Serial1.print("n6.val=");
    Serial1.print(B_max);
    Serial1.print("\xFF\xFF\xFF");
  }
  else if((int)encoder2.getDirection() != 0)
  {
    encoder_2_snapshot = (encoder2.getPosition() / 2);
    B_min += (encoder_2_snapshot - B_min_prev) * encoder_2_scale;
    B_min_prev = encoder_2_snapshot;
    Serial1.print("n7.val=");
    Serial1.print(B_min);
    Serial1.print("\xFF\xFF\xFF");
  }
}

void hall_probes_screen_run()
{
  int analogVal_1_prev = analogVal_1;
  int analogVal_2_prev = analogVal_2;
  if(millis() >= (time_now_adc + adc_period))
  {
    //time_now_adc += adc_period;
    time_now_adc = millis();
    
    if(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) // only if steppers have stopped update display
    {
      analogReadResolution(12);
      analogVal_1 = analogRead(analogPin_1);
      analogVal_2 = analogRead(analogPin_2);
      analogVal_1 = HALL_FILTER_ONE(analogVal_1_prev, 2, analogVal_1);
      analogVal_2 = HALL_FILTER_ONE(analogVal_2_prev, 2, analogVal_2);
      Serial1.print("n4.val=");
      Serial1.print((analogVal_1 - 1644) * 3300 / 4096); // display in mV (mT?)
      Serial1.print("\xFF\xFF\xFF");
      Serial1.print("n5.val=");
      Serial1.print((analogVal_2 - 1656) * 3300 / 4096); // display in mV (mT?)
      Serial1.print("\xFF\xFF\xFF");
      Serial.println("Hall probes run");
    }
  }
}

void Zero_All()
{
  Stepper_1_Position = 0;
  Stepper_2_Position = 0;
  RotaryValue_1 = 0;
  RotaryValue_2 = 0;
  Display_RotaryValue_1 = 0;
  Display_RotaryValue_2 = 0;

  encoder1.setPosition(0);
  encoder2.setPosition(0);
  SavedEncoder_1_Value = 0;
  SavedEncoder_2_Value = 0;

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  AccelStepper_run(Stepper_1_Position, Stepper_2_Position, Encoder_Index, RESET);
}
