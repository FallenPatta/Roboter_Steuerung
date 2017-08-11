#define TICKTIMESSIZE 10

int left_ticks = 0;
int left_direction = 1;
long left_ticks_times[TICKTIMESSIZE];
int left_times_index = 0;

int right_ticks = 0;
int right_direction = 1;
long right_ticks_times[TICKTIMESSIZE];
int right_times_index = 0;

int tick_difference = 0;
float integral_tickcontrol = 0;

long stoptime = 0;
long stoptime2= 0;
bool driving_status = true;

int motor_speeds[2];

void tick_left_ISR() {
  if(millis() != left_ticks_times[(left_times_index-1)%TICKTIMESSIZE]){
    left_ticks += left_direction;
    left_ticks_times[left_times_index] = millis();
    left_times_index = (left_times_index+1)%TICKTIMESSIZE;
  }
}

void tick_right_ISR() {
  right_ticks += right_direction;
  right_ticks_times[right_times_index] = millis();
  right_times_index = (right_times_index+1)%TICKTIMESSIZE;
}

void setMotors(int val1, int val2){
  setMotor('A', val1);
  setMotor('B', val2);
}

void setMotor(char m, int value){
  unsigned int abs_value = abs(value);
  switch(m){
    case 'A':
    case 'a':
      if(value > 0)
      {
        left_direction = 1;
        digitalWrite(D3, HIGH);
      }
      else if(value < 0)
      {
        left_direction = -1;
        digitalWrite(D3, LOW);
      }
      else
      {
        
      }
      analogWrite(D1, abs_value);
    break;
    case 'B':
    case 'b':
      if(value > 0)
      {
        right_direction = 1;
        digitalWrite(D4, LOW);
      }
      else if(value < 0)
      {
        right_direction = -1;
        digitalWrite(D4, HIGH);
      }
      else
      {
        
      }
      analogWrite(D2, abs_value);
    break;
    default:
    break;
  }
}

void speedControl(float dT, float vSoll, float wSoll, float* output){
  
}


float left_tps = 0;
float right_tps = 0;
float iVal_left = 0;
float iVal_right = 0;
bool wheelSpeedControl(char wheel, float ticksPerSecond, float dT, float * correction, float clampAroundZero){
  float proportional = 20.0;
  float differential = 3.0;
  float integral = 0.001;

  if(clampAroundZero < 0){
    clampAroundZero = -1*clampAroundZero;
  }
  
  int wheelTicks = 0;
  float tps_last = 0;
  float iVal = 0;
  long * timesArray;
  int index;
  switch(wheel){
    case 'A':
    case 'a':
      tps_last = left_tps;
      wheelTicks = left_ticks;
      iVal = iVal_left;
      timesArray = left_ticks_times;
      index = left_times_index;
    break;
    case'B':
    case'b':
      tps_last = right_tps;
      wheelTicks = right_ticks;
      iVal = iVal_right;
      timesArray = right_ticks_times;
      index = right_times_index;
    break;
    default:
    break;
  }

  float tps = 1000.0f / ((float)(timesArray[(index-1)%TICKTIMESSIZE] - timesArray[(index)]) / (float)TICKTIMESSIZE); //tick_diff/dT;
  float dError = tps - tps_last;
  float error = ticksPerSecond - tps;

  float pVal = proportional * error;
  float dVal = differential * dError;
  iVal += integral * error;

  iVal = max(-70, min(70, iVal));

  *correction = max(-1*clampAroundZero, min(clampAroundZero, pVal - dVal + iVal));

  switch(wheel){
    case 'A':
    case 'a':
      iVal_left = iVal;
      left_tps = tps;
      Serial.print("pid:\t");
      Serial.println(*correction );
//    Serial.print(",\t\t");
//    Serial.print(error);
//    Serial.print(",\t");
//    Serial.println(*correction);
//    Serial.print(",\t\t");
//    Serial.print(pVal);
//    Serial.print(",\t");
//    Serial.print(dVal);
//    Serial.print(",\t");
//    Serial.println(iVal);
    break;
    case'B':
    case'b':
      right_tps = tps;
      iVal_right = iVal;
    break;
    default:
    break;
  }
  return true;
}

void ticksControl(int speed1, int speed2, int* output){

  float proportional = 7.0;
  float differential = 25.0;
  float integral = 0.02;

  int tick_diff = (left_ticks - right_ticks);
  int tick_diff_diff = tick_diff - tick_difference;
  tick_difference = tick_diff;

  float proportional_tickcontrol = proportional * tick_diff;
  integral_tickcontrol += integral * tick_diff;
  float differential_tickcontrol = differential * tick_diff_diff;

  float control_output = max(-200, min(200, proportional_tickcontrol - differential_tickcontrol + integral_tickcontrol));

  output[0] = max(0, min(1024, speed1 - control_output));
  output[1] = max(0, min(1024, speed2 + control_output));

      Serial.print("pid:\t");
    Serial.print(proportional_tickcontrol);
    Serial.print(",\t");
    Serial.print(differential_tickcontrol);
    Serial.print(",\t");
    Serial.print(integral_tickcontrol);
    Serial.print(",\t");
    Serial.print(output[0]);
    Serial.print(",\t");
    Serial.print(output[1]);
    Serial.print(",\t");
    Serial.print(tick_diff);
    Serial.print(",\t");
    Serial.println(tick_diff_diff);
}

void setup() {
  //pinMode(BUILTIN_LED, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, LOW);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(D5, INPUT);
  attachInterrupt(D5, tick_left_ISR, CHANGE);
  pinMode(D6, INPUT);
  attachInterrupt(D6, tick_right_ISR, CHANGE);

  Serial.begin(115200);
  for (int i = 0; i < 10; i++) {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    delay(100);
  }
  motor_speeds[0] = 950;
  motor_speeds[1] = 950;
  stoptime = millis();
  stoptime2 = stoptime;
  Serial.println("START");
  setMotors(1024, 1024);
  delay(100);
}

 int relative_position = 500;
 bool forward = true;
void loop() {
//
//  if(millis() - stoptime > 20)
//  {
//    ticksControl(650, 650, motor_speeds);
//    setMotors(motor_speeds[0], motor_speeds[1]);
//    stoptime+=20;
//  }
  
  if(millis() - stoptime2 > 150)
  {
    int speeds = 750;
    float * cVal = new float;
    if(wheelSpeedControl('A', 50, 150.0f/1000.0f, cVal, 200)){
      setMotor('A',(int)(speeds+(*cVal)));
    }
    if(wheelSpeedControl('B', 50, 150.0f/1000.0f, cVal, 200)){
      setMotor('B',(int)(speeds+(*cVal)));
    }
    delete cVal;
    stoptime+=150;
  }

//  for(int i = 0; i<TICKTIMESSIZE; i++){
//    Serial.println(left_ticks_times[(left_times_index + i)%TICKTIMESSIZE]);
//  }
//  Serial.println("\n\n");

}
