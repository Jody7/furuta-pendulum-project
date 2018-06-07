#define NO_PORTC_PINCHANGES

#include "Encoder.h"
#include "RunningAverage.h"
#include "definitions.h"
#include "variables.h"
#include "eRCaGuy_Timer2_Counter.h"
#include "fastMathRoutines.h"
#include "PinChangeInt.h"
#include "RCdecode.h"

#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)

#define PWM_A_MOTOR1 3
#define PWM_B_MOTOR1 5
#define PWM_C_MOTOR1 6

#define PWM_A_MOTOR2 9 
#define PWM_B_MOTOR2 10
#define PWM_C_MOTOR2 11

const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};;
 
int currentStepA;
int currentStepB;
int currentStepC;
int sineArraySize;
int phaseShift;

volatile long encoderCount = 0;

volatile uint8_t pwm_a;
volatile uint8_t pwm_b;
volatile uint8_t pwm_c;

byte PWM_PIN = 14;
float currentVirtPos = 99999;
double velocity = 0.0000;
double accel = 0.0000;
float desiredPos = 0;

const byte INPUT_PIN = 14; 

byte input_pin_bitMask;
volatile byte* p_input_pin_register;

byte bitMaskA;
volatile byte* chan_a_register;

byte bitMaskB;
volatile byte* chan_b_register;
 
volatile unsigned long pulseCounts = 0;
volatile boolean pwm_data = false;
int loop_count = 0;

RunningAverage averageMotorEncoder(2);
RunningAverage averagePendD(3);
RunningAverage motorEncoderD(5);
RunningAverage pendEncoder(1);


#define ENCODER_ROLLOVER_SENS 300
int encoder_rollover_cooldown = 0;
int encoder_rollover = 1; //start at one because first tick rolls it
float last_pwm = 0;
float encoder_value = 0;
float operating_value = 0;

//Encoder pendEncoder(15, 16);
long oldPendEncoder = 0;

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

#define TOTAL_LOOP_TIME 20000

void setup() {
  Serial.begin(115200);
  
  initBGC();
  initRCPins();

  pinMode(PWM_PIN, INPUT);

  bitMaskA = digitalPinToBitMask(15);
  chan_a_register = portInputRegister(digitalPinToPort(15));

  bitMaskB = digitalPinToBitMask(16);
  chan_b_register = portInputRegister(digitalPinToPort(16));
  
  input_pin_bitMask = digitalPinToBitMask(INPUT_PIN);
  p_input_pin_register = portInputRegister(digitalPinToPort(INPUT_PIN));

  timer2.setup();
  
  //configurePinChangeInterrupts();
  pciSetup(14);
  pciSetup(15);
  pciSetup(16);

  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = 0;
  currentStepC = 0;
 
  sineArraySize--;
}

boolean bloop = false;

short loopcount = 0;
int offsetPendulum = 0;
int lastPendValue = 0;
float lastEncoderValue = 0;
void loop() {
  int pendValue;
  
  if (Serial.available() > 0) {
    if(Serial.read() == 'r') {
      offsetPendulum = encoderCount;
    }
    if(operating_value == 0) {
      operating_value = 1;
    } else {
      operating_value = 0;
    }
    
    lastPendValue = 0;
  }
  
  long newPendEncoder = encoderCount;
  pendValue = newPendEncoder - offsetPendulum;
  
  loopcount++;
  //unsigned long start_time = micros();
  
  if (pwm_data == true) {
    static float pulseTime = 0; //us; the most recent input signal high pulse time
    static float pd_us = 0; //us; the most recent input signal period between pulses
    static float pulseFreq = 0; //Hz, the most recent input signal pulse frequency
    
    pwm_data = false;
    unsigned long pulseCountsCopy = pulseCounts;
    interrupts();
    
    //do calculations
    pulseTime = pulseCountsCopy/2.0;

    if(encoder_rollover_cooldown > 0) {
          encoder_rollover_cooldown--;
    }
        
    if(pulseTime < 3000) {
      if(pulseTime < 1000) {
        averageMotorEncoder.addValue(pulseTime);
        float pwm_avg = averageMotorEncoder.getAverage();
        encoder_value = pwm_avg + (encoder_rollover * 925);
        
        if(encoder_rollover_cooldown <= 0) {
          if(pulseTime - last_pwm > ENCODER_ROLLOVER_SENS) {
            encoder_rollover_cooldown = encoder_rollover_cooldown + 100;
            encoder_rollover--;
          } else if(pulseTime - last_pwm < -ENCODER_ROLLOVER_SENS) {
            encoder_rollover_cooldown = encoder_rollover_cooldown + 100;
            encoder_rollover++;
          }
        }
        
        last_pwm = pulseTime;
      }
    }
    
  }

  averagePendD.addValue(-(pendValue - lastPendValue) * 50);
  motorEncoderD.addValue((encoder_value - lastEncoderValue) * 50);

  if((bloop == false) && (abs(pendValue) < 10)) {
    bloop = true;
  } else if(abs(pendValue) > 1000) {
    bloop = false;
  }

  if(operating_value != 1 && (abs(pendValue) < 1000) && bloop) {
    float kalpha = 5.3488;
    float kalphaDot = 0.6575;
    float kthetaDot = -0.2078;

    if(abs(pendValue) > 30) {
      //kalpha = 10;
      //kalphaDot = 1.25;
      //kthetaDot = 0.25;

      kalpha = 7.25;
      kalphaDot = 4;
      kthetaDot = 1.5;
    } else {
      //under 10 ticks
      kalpha = 5.3488;
      kalphaDot = 0.6575;
      kthetaDot = -0.2078;
    }

    accel = (averagePendD.getAverage() * kalphaDot) + (-(pendValue) * kalpha);
    accel = accel + (motorEncoderD.getAverage() * kthetaDot);
    accel = -accel * 0.0000197;
  } else{
    accel = 0;
    velocity = 0;
  }

  
  velocity = velocity + accel;

  if(velocity > 5) {
    velocity = 5;
  } else if(velocity < -5) {
    velocity = -5;
  }

  lastPendValue = pendValue;
  lastEncoderValue = encoder_value;

  currentVirtPos = currentVirtPos + velocity;

  if(currentVirtPos > sineArraySize) {
    currentVirtPos = 0;
  } else if(currentVirtPos < 0) {
    currentVirtPos = sineArraySize;
  }
  
  if(operating_value != 1 && bloop) {
    setMotorPosition(2, currentVirtPos, 200);
  } else {
    setMotorPosition(2, currentVirtPos, 5);
  }

  //unsigned long end_time = micros();
  //unsigned long remain_time = (TOTAL_LOOP_TIME - (end_time - start_time)) + end_time;

  /*
  while(remain_time > micros()) {
    //waste proccessor cycles
    //Serial.println("waiting");
  }
  */
  
  if(loopcount > 100) {
    Serial.println(pendValue);
    //Serial.println(encoder_value);
    //Serial.println(velocity, 6);
    //Serial.println(accel, 6);
    loopcount = 0;
  }

  //Serial.println(encoder_value);
  //Serial.println(encoder_rollover);
}

void setMotorPosition(int motor,unsigned int position, int power) {
  currentStepA = position;
  currentStepB = position + phaseShift;
  currentStepC = position + phaseShift + phaseShift;

  currentStepA = currentStepA % sineArraySize;
  currentStepB = currentStepB % sineArraySize;
  currentStepC = currentStepC % sineArraySize;

  pwm_a = pwmSin[currentStepA] * power/255;
  pwm_b = pwmSin[currentStepB] * power/255;
  pwm_c = pwmSin[currentStepC] * power/255;

  analogWrite(PWM_C_MOTOR1, pwm_a);
  analogWrite(PWM_A_MOTOR2, pwm_b);
  analogWrite(PWM_B_MOTOR2, pwm_c);
}

void initBGC() {
  // sets the speed of PWM signals. 
  TCCR0B = TCCR0B & 0b11111000 | 0x01;
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  pinMode(PWM_A_MOTOR1, OUTPUT); 
  pinMode(PWM_B_MOTOR1, OUTPUT); 
  pinMode(PWM_C_MOTOR1, OUTPUT); 
  
  pinMode(PWM_A_MOTOR2, OUTPUT); 
  pinMode(PWM_B_MOTOR2, OUTPUT); 
  pinMode(PWM_C_MOTOR2, OUTPUT); 
}

void pinChangeIntISR()
{  
  //local variables
  static boolean pin_state_new = LOW; //initialize
  static boolean pin_state_old = LOW; //initialize
  static unsigned long t_start = 0; //units of 0.5us
  static unsigned long t_start_old = 0; //units of 0.5us
  
  pin_state_new = fastDigitalRead(p_input_pin_register,input_pin_bitMask);
  if (pin_state_old != pin_state_new)
  {
    //if the pin state actualy changed, & it was not just noise lasting < ~2~4us
    pin_state_old = pin_state_new; //update the state
    if (pin_state_new == HIGH)
    {
      t_start = timer2.get_count(); //0.5us units
      t_start_old = t_start; //0.5us units; update
      pwm_data = true;
    }
    else //pin_state_new == LOW
    {
      unsigned long t_end = timer2.get_count(); //0.5us units
      pulseCounts = t_end - t_start; //0.5us units
    }
  }
}

ISR(PCINT1_vect)
{
  //Serial.println("beep");
  encoderStuff();
  pinChangeIntISR();
}

volatile boolean a_p;
volatile boolean b_p;
volatile boolean a;
volatile boolean b;

void encoderStuff() {
    boolean a = fastDigitalRead(chan_a_register, bitMaskA);
    boolean b = fastDigitalRead(chan_b_register, bitMaskB);

    encoderCount = encoderCount + ParseEncoder(a, b);
    
    a_p = a;
    b_p = b;
}

int ParseEncoder(boolean a, boolean b){
  if(a_p && b_p){
    if(!a && b) return 1;
    if(a && !b) return -1;
  }else if(!a_p && b_p){
    if(!a && !b) return 1;
    if(a && b) return -1;
  }else if(!a_p && !b_p){
    if(a && !b) return 1;
    if(!a && b) return -1;
  }else if(a_p && !b_p){
    if(a && b) return 1;
    if(!a && !b) return -1;
  }
  return 0;
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

/*
void configurePinChangeInterrupts()
{
  volatile byte* p_PCMSK = (volatile byte*)digitalPinToPCMSK(INPUT_PIN); //pointer to the proper PCMSK register
  *p_PCMSK |= _BV(digitalPinToPCMSKbit(INPUT_PIN));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR = (volatile byte*)digitalPinToPCICR(INPUT_PIN); //pointer to PCICR
  *p_PCICR |= _BV(digitalPinToPCICRbit(INPUT_PIN));
}
*/

