#include <FlashStorage.h>
#include <AccelStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, 10, 9); // (10 = step) ; (9 = dir)
AccelStepper stepper2(AccelStepper::DRIVER, 21, 20); // (21 = step) ; (20 = dir)

#define enc_dt       13
#define enc_clk      12
#define enc_sw        5 // was on pin 11 (interrupt 0) caused crashing

#define step_pin     10
#define dir_pin       9

#define CCW           0
#define CW            1
#define PULSE_WIDTH                     500
#define SPEED                          4000 // Stepper speed in steps per second
#define RUNNING                           1
#define STOPPED                           0

#define NO_INIT                           0
#define INIT                              1

#define FALSE                             0
#define TRUE                              1

#define MOTOR_STEPS_PER_ROTATION        200
#define GEAR_RATIO                      100
#define GEARED_STEPS_PER_ROTATION     20000

#define HOME_SCREEN_INIT                  0
#define HOME_SCREEN                       1
#define DATA_INPUT_SCREEN_INIT            2
#define DATA_INPUT_SCREEN                 3

#define BUTTON_ID_03                      3
#define BUTTON_ID_04                      4
#define BUTTON_ID_06                      6
#define BUTTON_ID_07                      7

int screenState = HOME_SCREEN_INIT; // start at home screen init state

volatile int encoderValue = 0;
volatile int turnDetected = 0;
volatile int pinSwitchDetected = 0;

int Bx;
int By;

int StepperPosition = 0;
int PinSwitchFlag = 0;
int SavedEncoderValue = 0;
int motorDirectionChange = FALSE;

typedef struct
{
  int nvm_motor_position;
  int nvm_encoder_value;
  int nvm_saved_encoder_value;
  int nvm_Bx;
  int nvm_By;
}storage;

FlashStorage(flash_store, storage);
storage flash_variables;

int pos = 3600;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);

  pinMode(enc_dt, INPUT_PULLUP);  
  pinMode(enc_clk, INPUT_PULLUP);
  pinMode(enc_sw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_clk), encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc_sw), pin_switch, FALLING);

  stepper1.setMaxSpeed(SPEED);
  stepper1.setAcceleration(10000);
  stepper2.setMaxSpeed(SPEED);
  stepper2.setAcceleration(10000);
  
  restoreNVM(); // read nvm and restore stored variables
}

void loop()
{
  switch(screenState)
  {
    case HOME_SCREEN_INIT:
      encoderValue = SavedEncoderValue;
      updateDisplay(StepperPosition, INIT);          // Update display at initialisation
      screenState = HOME_SCREEN;
      break;
    
    case HOME_SCREEN:
      readRotaryEncoder();                           // Read rotary encoder for distance and direction
      updateDisplay(StepperPosition, NO_INIT);       // Update display before move to new position
      AccelStepper_run(StepperPosition);
      displayRun();
      break;

    case DATA_INPUT_SCREEN_INIT:                     // state to enter once before main state
      data_input_screen_run(INIT);
      screenState = DATA_INPUT_SCREEN;
      break;
    
    case DATA_INPUT_SCREEN:
      data_input_screen_run(NO_INIT);
      displayRun();
      break;
  }
}

void readRotaryEncoder()
{
  int RotaryValue = 0;

  RotaryValue = encoderValue * GEAR_RATIO;

  if(RotaryValue >= GEARED_STEPS_PER_ROTATION)
  {
    RotaryValue = GEARED_STEPS_PER_ROTATION;
    encoderValue = MOTOR_STEPS_PER_ROTATION; // encoder counts in raw motor steps (not geared steps)
  }
  else if(RotaryValue <= 0)
  {
    RotaryValue = 0;
    encoderValue = 0;
  }
  else
  {
    //do nothing
  }

  StepperPosition = RotaryValue;
  SavedEncoderValue = encoderValue;
}

void AccelStepper_run(int stepperPos)
{
  int stepper_1_pos = stepperPos;
  int stepper_2_pos = stepperPos;
  static int motorDirectionChangePrev = 0;

  if(pinSwitchDetected == 1)
  {
    pinSwitchDetected = 0;

    if(motorDirectionChange == TRUE && motorDirectionChangePrev == FALSE)
    {
      stepper2.setCurrentPosition(-stepper2.currentPosition());
      motorDirectionChangePrev = motorDirectionChange;
    }
    else if(motorDirectionChange == FALSE && motorDirectionChangePrev == TRUE)
    {
      stepper2.setCurrentPosition(-stepper2.currentPosition());
      motorDirectionChangePrev = motorDirectionChange;
    }
    if(motorDirectionChange == TRUE)
    {
      stepper_2_pos = -stepperPos;  // run stepper 2 in opposite direction to stepper 1
    }
    else if(motorDirectionChange == FALSE)
    {
      stepper_2_pos = stepperPos;  // run stepper 2 in opposite direction to stepper 1
    }
    
    stepper1.moveTo(stepper_1_pos);
    stepper2.moveTo(stepper_2_pos);
    displayMotorStatus(RUNNING);
  }
  stepper1.run();
  stepper2.run();
  if(stepper1.distanceToGo() == 0)
  {
    displayMotorStatus(STOPPED);
  }
}

void encoder()
{
  turnDetected = 1;
  (digitalRead(enc_clk) == digitalRead(enc_dt)) ? (encoderValue--) : (encoderValue++);
}

void updateDisplay(int stepper_pos, int init)
{
  static int currentValue = 0;
  int displayValue = 0;
  //displayValue = (stepper_pos * 360 / 20000);

  displayValue = stepper_pos;

  if((currentValue != displayValue) || (init == 1))
  {
    Serial1.print("n0.val=");
    Serial1.print(displayValue);
    Serial1.print("\xFF\xFF\xFF");
  }
  currentValue = displayValue;
}

void displayRun()
{
  byte received_data[7];
  for(int i = 0; i < 7; i++)
  {
    received_data[i] = 0;
  }
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
  StepperPosition = flash_variables.nvm_motor_position;             // restore stepper position
  encoderValue = flash_variables.nvm_encoder_value;                 // restore encoder value
  SavedEncoderValue = flash_variables.nvm_saved_encoder_value;      // restore saved encoder value
  Bx = flash_variables.nvm_Bx;                                      // restore Bx
  By = flash_variables.nvm_By;                                      // restore By
}

void saveToFlash()
{
  flash_variables.nvm_motor_position = StepperPosition;             // save stepper position
  flash_variables.nvm_encoder_value = encoderValue;                 // save encoder value
  flash_variables.nvm_saved_encoder_value = SavedEncoderValue;      // save backup of encoder value
  flash_variables.nvm_Bx = Bx;                                      // save Bx
  flash_variables.nvm_By = By;                                      // save By
  flash_store.write(flash_variables);                               // save structure
}

void data_input_screen_run(int init)
{
  static int Bx_previous_value = 0;
  static int By_previous_value = 0;
  int encoder_snapshot = 0;

  if(init)
  {
    Bx_previous_value = 0;
    By_previous_value = 0;
    SavedEncoderValue = encoderValue;
    encoderValue = 0;
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
    encoderValue = 0;
    Bx_previous_value = 0;
    By_previous_value = 0;
    turnDetected = 1;
  }

  if(turnDetected)
  {
    turnDetected = 0;
    encoder_snapshot = encoderValue;

    if(PinSwitchFlag == 0)
    {
      Serial.println(encoderValue);
      Serial.println(Bx_previous_value);
      Serial.println(Bx);
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
      Serial.println(encoderValue);
      Serial.println(By_previous_value);
      Serial.println(By);
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
  SavedEncoderValue = encoderValue;
  encoderValue = 0;
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
