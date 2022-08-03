#include <FlashStorage.h>

#define enc_dt 6
#define enc_clk 5
#define enc_sw 11
#define CCW 0
#define CW 1
#define PULSE_WIDTH                     500

#define MOTOR_STEPS_PER_ROTATION        200
#define GEAR_RATIO                      100
#define GEARED_STEPS_PER_ROTATION     20000

#define HOME_SCREEN             0
#define DATA_INPUT_SCREEN_ENTER 1
#define DATA_INPUT_SCREEN       2

int screenState = HOME_SCREEN;

volatile int encoderValue = 0;
volatile int turnDetected = 0;
volatile int pinSwitchDetected = 0;

byte received_data[7];

int Distance = 0;
int StepperPosition = 0;
int StepsToMove = 0;
int PinSwitchFlag = 0;
int StepsToTake = 10;

typedef struct
{
  int nvm_motor_position;
  int nvm_encoder_value;
  int Bx;
  int By;
}storage;

FlashStorage(flash_store, storage);
storage flash_variables;



void setup()
{
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(115200);
  pinMode(10, OUTPUT); // Step
  pinMode(9, OUTPUT); // Direction
  digitalWrite(10, LOW); // 2 = 9 = step
  digitalWrite(9, LOW); // 3 = 8 = dir

  pinMode(enc_dt, INPUT_PULLUP);  
  pinMode(enc_clk, INPUT_PULLUP);
  pinMode(enc_sw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_clk), encoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc_sw), pin_switch, FALLING);
    
  
  restoreNVM(); // read nvm and restore stored variables
}

void loop()
{
  switch(screenState)
  {
    case HOME_SCREEN:
      moveStepper();
      updateDisplay();
      displayRun();
      break;

    case DATA_INPUT_SCREEN_ENTER: // state to enter once before main state
      setValueFocus(0);
      screenState = DATA_INPUT_SCREEN;
      break;
    
    case DATA_INPUT_SCREEN:
      increment_number();
      displayRun();
      break;
  }
}

void moveStepper()
{
  int RotaryValue = 0;
  int Direction = 0; // 0 = CCW; 1 = CW

  if(turnDetected)
  {
    turnDetected = 0;
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
  
    if(RotaryValue >= StepperPosition)
    {
      StepsToMove = (RotaryValue - StepperPosition);
      Direction = 1;
      //Serial.println(RotaryValue);
      //Serial.println(StepperPosition);
    }
    else if(RotaryValue <= StepperPosition)
    {
      StepsToMove = (StepperPosition - RotaryValue);
      Direction = 0;
      //Serial.println(RotaryValue);
      //Serial.println(StepperPosition);
    }
    else
    {
      //do nothing
    }
  
    if(StepsToMove != 0)
    {
      //Serial.println(StepsToMove);
      StepperSetDirection(Direction);
      for(int i=0; i<StepsToMove; i++) //1/16 steps = 3200 steps per rev
      {
        //for(int j=0; j<StepsToTake; j++)
        //{
          digitalWrite(10, HIGH); // Step at 1ms
          delayMicroseconds(PULSE_WIDTH);
          digitalWrite(10, LOW); // Step at 1ms
          delayMicroseconds(PULSE_WIDTH);
        //}
      }
    }
    StepperPosition = RotaryValue;
    Serial.println(StepperPosition);
  }
}




void encoder()
{
  turnDetected = 1;
  delayMicroseconds(150);
  if(digitalRead(enc_clk) == digitalRead(enc_dt))
  {
    encoderValue--;
  }
  else
  {
    encoderValue++;
  }
}

void updateDisplay()
{
  static int currentValue = 0;
  int displayValue;
  displayValue = (StepperPosition * 360 / 20000);
  if(currentValue != displayValue)
  {
    Serial1.print("n0.val=");
    Serial1.print(displayValue);
    Serial1.print("\xFF\xFF\xFF");
  }
  currentValue = displayValue;
}

void displayRun()
{
  while(Serial1.available() > 0)
  {
    Serial1.readBytesUntil('\n', received_data, 7);
    if(received_data[2] == 0x03) // position 2 has the button ID which is 0x03
    {
      Serial.println("Store was pressed!");
      saveToFlash();
    }
    else if(received_data[2] == 0x04) // position 2 has the 2nd button ID which is 0x04
    {
      //button pg2 has been pressed - this is data entry screen
      screenState = DATA_INPUT_SCREEN_ENTER;
    }
    else if(received_data[2] == 0x07) // position 2 has the page 2 back button ID which is 0x07
    {
      //pg2 back has been pressed - go back to home screen
      screenState = HOME_SCREEN;
    }
  }
}

void StepperSetDirection(int Direction)
{
  if(Direction)
  {
    digitalWrite(9, LOW);
  }
  else
  {
    digitalWrite(9, HIGH);
  }
}

void pin_switch()
{
  pinSwitchDetected = 1;
  delayMicroseconds(2000);
  if(PinSwitchFlag == 1)
  {
    PinSwitchFlag = 0;
  }
  else
  {
    PinSwitchFlag = 1;
  }
}


void run_to_home()
{
  while(StepperPosition != 0)
  {
    Serial.println(StepperPosition);
    StepperSetDirection(CCW);
    digitalWrite(10, HIGH); // Step at 1ms
    delayMicroseconds(PULSE_WIDTH);
    digitalWrite(10, LOW); // Step at 1ms
    delayMicroseconds(PULSE_WIDTH);
    StepperPosition = StepperPosition - 1;
  }
}

void restoreNVM()
{
  flash_variables = flash_store.read();
  StepperPosition = flash_variables.nvm_motor_position; // restore stepper position
  encoderValue = flash_variables.nvm_encoder_value;     // restore encoder value
}

void saveToFlash()
{
  flash_variables.nvm_motor_position = StepperPosition;
  flash_variables.nvm_encoder_value = encoderValue;
  flash_store.write(flash_variables);
}


void increment_number()
{
  int Bx_val_to_send = 0;
  int By_val_to_send = 0;
  static int Bx_previous_value = 0;
  static int By_previous_value = 0;
  int encoder_snapshot = 0;

  if(turnDetected || pinSwitchDetected)
  {
    encoder_snapshot = encoderValue;
    turnDetected = 0;
    pinSwitchDetected = 0;
    if(PinSwitchFlag == 0)
    {
      Serial.println(encoderValue);
      Serial.println(Bx_previous_value);
      setValueFocus(0);
      flash_variables.Bx += (encoder_snapshot - Bx_previous_value);
      Bx_previous_value = encoder_snapshot;
      By_previous_value = encoder_snapshot;
      Serial1.print("n0.val=");
      Serial1.print(flash_variables.Bx);
      Serial1.print("\xFF\xFF\xFF");
    }
    else
    {
      encoder_snapshot = encoderValue;
      setValueFocus(1);
      flash_variables.By += (encoder_snapshot - By_previous_value);
      Bx_previous_value = encoder_snapshot;
      By_previous_value = encoder_snapshot;
      Serial1.print("n1.val=");
      Serial1.print(flash_variables.By);
      Serial1.print("\xFF\xFF\xFF");
    }
  }
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
