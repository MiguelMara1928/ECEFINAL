#include <ECE3.h>


// TUNE SPEED, UTURN SPIN, WITH NEW BATTERIES


// pin declarations
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;
uint16_t sensorValues[8]; 

// calibration constants
const int weights[] = {-8, -4, -2, -1, 1, 2, 4, 8};
const int sensorMins[8] = {803, 712,  595,  641,  595,  664,  641,  781};
const float sensorRange[8] = {1697, 1442, 1364, 976,  1203, 1836, 1859, 1719};

// tuning
int baseSpeed = 120; 
float Kp = 0.07;    
float Kd = 0.6; 

// globals
int lastError = 0;
int barCounter = 0;    
bool turnTriggered = false;
bool secondLoop = false;



void setup() {
  ECE3_Init();
  Serial.begin(9600);
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW); 
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  delay(2000);
}

// normalizing sensor value function
float fusionOutput(float s, int i){
  float val = ((s - sensorMins[i])/sensorRange[i])*1000;
  return constrain(val, 0, 1000);
}

// uturn check function
bool UturnCheck() {
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) {
    if (fusionOutput(sensorValues[i], i) > 100) {
      whiteCount++;
    }
  }
  return (whiteCount >= 6);
}

void Uturn(){

  if (turnTriggered) {  //only executes if triggered

    // stop on the second loop around
    if (secondLoop) {
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      while(true);
    }



    
    // uturn hardcode
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      delay(10);
      
      digitalWrite(right_dir_pin, HIGH); 
      analogWrite(left_pwm_pin, 215);
      analogWrite(right_pwm_pin, 215);
      delay(250); //. <------------ TUNE WITH NEW BATTERIES
      
      digitalWrite(right_dir_pin, LOW);
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      delay(10);

      analogWrite(left_pwm_pin, 150);
      analogWrite(right_pwm_pin, 150);
      delay(240);

      // reset everything
      turnTriggered = false; 
      barCounter = 0;
      secondLoop = true;
      lastError = 0;
  }
}

int Xspeed(int cur, int der) {
  //dynamic speed
  // if error is small and derivative is small go fast otherwise go slow
  if (abs(cur) < 145 && abs(der) < 35){
    Kd = 0.8;
    return 235; // straightaway speed
  }
  Kd = 0.75;
  return 200;  // cornering speed
}

void loop() {

  ECE3_read_IR(sensorValues);

 
  if (UturnCheck()) {
    barCounter++;
  } else {
    barCounter = 0; // If any loop sees white reset
  }

  
  if (barCounter >= 4) {   //check twice before trigger (anti-phantom reading)
    turnTriggered = true;
  }

  // error calculation
  float currentError = 0;
  for (int i = 0; i < 8; i++) {
    currentError += weights[i] * fusionOutput((float)sensorValues[i], i);
  }
  currentError = currentError / 4.0;
  int derivative = currentError - lastError;

  //dynamic speed (gain scheduling) 
  int targetSpeed = Xspeed(currentError, derivative);

  //acceleration control: changes speed gradually
  if (baseSpeed < targetSpeed)
    baseSpeed += 5;
  else if (baseSpeed > targetSpeed)
    baseSpeed -= 5;

  baseSpeed = constrain(baseSpeed, 200, 235);


  // PD Logic

  int adjustment = (Kp * currentError) + (Kd * derivative);
  lastError = currentError;

  // calculate Motor Speeds
  int leftSpd  = baseSpeed - adjustment;
  int rightSpd = baseSpeed + adjustment;
  
  leftSpd  = constrain(leftSpd, 0, 255);
  rightSpd = constrain(rightSpd, 0, 255);

  // write to motors
  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

  // only executes Uturn if its triggered
  Uturn();
}