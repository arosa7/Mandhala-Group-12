
const int ENC_OUTER = A0; 
const int ENC_INNER = A1; 

const int EN_OUTER   = 3;  
const int STEP_OUTER = 4;  
const int DIR_OUTER  = 5;  // <----- Pin layouts

const int EN_INNER   = 8;  
const int STEP_INNER = 9;  
const int DIR_INNER  = 10; 


const float STEPS_PER_DEGREE = 26.666; 
//calculated by 3200 microsteps * 3 (full rotations cause 3:1) = 9600 / 360 = 26.66

float prevRawOuter = 0;
float accumMotorOuter = 0;
float ringOffsetOuter = 0;

float prevRawInner = 0;
float accumMotorInner = 0;
float ringOffsetInner = 0;

float targetAngleOuter = 0;
float targetAngleInner = 0;
bool outerActive = false;
bool innerActive = false;

const float DEADBAND = 1.5; //tolerance because magnets are not perfect

void updateEncoders() {
  
  float rawO = (analogRead(ENC_OUTER) / 1023.0) * 360.0;
  float deltaO = rawO - prevRawOuter;
  if (deltaO < -180.0){
    deltaO += 360.0; 
  }
  if (deltaO > 180.0){
    deltaO -= 360.0;
  }
  accumMotorOuter += deltaO;
  prevRawOuter = rawO;

 
  float rawI = (analogRead(ENC_INNER) / 1023.0) * 360.0;
  float deltaI = rawI - prevRawInner;
  if (deltaI < -180.0){
    deltaI += 360.0;
  } 
  if (deltaI > 180.0){ 
    deltaI -= 360.0;  
  }
  accumMotorInner += deltaI;
  prevRawInner = rawI;
}

float getRingAngleOuter() {
  float angle = (accumMotorOuter / 3.0) - ringOffsetOuter;
  while(angle >= 360.0){
    angle -= 360.0;
  }
  while(angle < 0.0){
    angle += 360.0;
  }
  return angle;
}

float getRingAngleInner() {
  float angle = (accumMotorInner / 3.0) - ringOffsetInner;
  while(angle >= 360.0){
    angle -= 360.0;
  } 
  while(angle < 0.0){
   angle += 360.0; 
  } 
  return angle;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(EN_OUTER, OUTPUT); 
  pinMode(STEP_OUTER, OUTPUT); 
  pinMode(DIR_OUTER, OUTPUT);
  pinMode(EN_INNER, OUTPUT); 
  pinMode(STEP_INNER, OUTPUT); 
  pinMode(DIR_INNER, OUTPUT);
  
  digitalWrite(EN_OUTER, HIGH);
  digitalWrite(EN_INNER, HIGH); // <----- turns off motors so not noisy

  delay(500); 
  
  prevRawOuter = (analogRead(ENC_OUTER) / 1023.0) * 360.0;
  accumMotorOuter = prevRawOuter;
  ringOffsetOuter = accumMotorOuter / 3.0;

  prevRawInner = (analogRead(ENC_INNER) / 1023.0) * 360.0;
  accumMotorInner = prevRawInner;
  ringOffsetInner = accumMotorInner / 3.0;

  Serial.println("System Ready. Infinite 3:1 Tracking Active.");
  Serial.println("Type 'O180' to move Outer. Type 'I90' to move Inner.");
}

void loop() {
  updateEncoders(); // Keep memory fresh

  // 1. Handle Commands
  if (Serial.available() > 0) {
    char ring = Serial.read();
    int angle = Serial.parseInt();
    
    if (angle < 0) angle = 0;
    if (angle > 359) angle = 359; 

    if (ring == 'O' || ring == 'o') {
      targetAngleOuter = angle;
      outerActive = true;   
      digitalWrite(EN_OUTER, LOW); 
      Serial.print("\n>>> Commanding Outer to: "); Serial.println(angle);
    } 
    else if (ring == 'I' || ring == 'i') {
      targetAngleInner = angle;
      innerActive = true;   
      digitalWrite(EN_INNER, LOW); 
      Serial.print("\n>>> Commanding Inner to: "); Serial.println(angle);
    }
  }

  
  if (outerActive) {
    float currentRingAngle = getRingAngleOuter();
    float error = targetAngleOuter - currentRingAngle;
    
    
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if (abs(error) > DEADBAND) {
      
      long stepsNeeded = abs(error) * STEPS_PER_DEGREE;
      bool dir = (error > 0) ? HIGH : LOW; 
      
      takeContinuousSteps(DIR_OUTER, STEP_OUTER, dir, stepsNeeded);
    } else {
      outerActive = false; 
      digitalWrite(EN_OUTER, HIGH); 
      Serial.print("--- Outer Finished! Ring is exactly at: ");
      Serial.print(getRingAngleOuter());
      Serial.println("° ---");
    }
  }

  
  if (innerActive) {
    float currentRingAngle = getRingAngleInner();
    float error = targetAngleInner - currentRingAngle;
    
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if (abs(error) > DEADBAND) {
      
      long stepsNeeded = abs(error) * STEPS_PER_DEGREE;
      bool dir = (error > 0) ? HIGH : LOW; 
      
      takeContinuousSteps(DIR_INNER, STEP_INNER, dir, stepsNeeded);
    } 
    else {
      innerActive = false; 
      digitalWrite(EN_INNER, HIGH); 
      Serial.print("--- Inner Finished! Ring is exactly at: ");
      Serial.print(getRingAngleInner());
      Serial.println("° ---");
    }
  }
}


void takeContinuousSteps(int dirPin, int stepPin, bool direction, long steps) {
  digitalWrite(dirPin, direction);
  
  for(long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200); 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200); 

    if (i % 20 == 0) {
      updateEncoders(); 
    }
  }
}
