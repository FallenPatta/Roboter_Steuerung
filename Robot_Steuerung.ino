#define TICKTIMESSIZE 10

//Alle Abmessungen in cm
#define WHEEL_DIAMETER 7.0f
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0f
#define WHEEL_CIRC  WHEEL_DIAMETER * PI
#define TICK_LEN  WHEEL_CIRC/48.0f

#define ROBOT_WIDTH 13.5f
#define ROBOT_WIDTH_2 ROBOT_WIDTH/2.0f

#define TICK_TRUST 0.2f

//typedef void (*VarFunc)(void);
//
//typedef struct {
//  VarFunc function;
//  long next_tick;
//  long tick_duration;
//} TickFunction;
//
//TickFunction * functions;


//Status Variables
int left_ticks = 0;
int left_direction = 1;
long left_ticks_times[TICKTIMESSIZE];
int left_times_index = 0;

int right_ticks = 0;
int right_direction = 1;
long right_ticks_times[TICKTIMESSIZE];
int right_times_index = 0;

float wheel_speeds[2];


//Control Variables
int tick_difference = 0;
float integral_tickcontrol = 0;

long stoptime = 0;
long stoptime2 = 0;
bool driving_status = true;

int motor_speeds[2];

void tick_left_ISR() {
  long tickTime = millis();
  if(tickTime != left_ticks_times[(left_times_index-1)%TICKTIMESSIZE] && tickTime != left_ticks_times[(left_times_index-1)%TICKTIMESSIZE]+1){
    left_ticks += left_direction;
    left_ticks_times[left_times_index] = tickTime;
    left_times_index = (left_times_index + 1) % TICKTIMESSIZE;
  }
}

void tick_right_ISR() {
  long tickTime = millis();
  if(tickTime != right_ticks_times[(right_times_index-1)%TICKTIMESSIZE] && tickTime != right_ticks_times[(right_times_index-1)%TICKTIMESSIZE]+1){
    right_ticks += right_direction;
    right_ticks_times[right_times_index] = tickTime;
    right_times_index = (right_times_index + 1) % TICKTIMESSIZE;
  }
}

/*      if(left_ticks_times[(left_times_index-1)%TICKTIMESSIZE] != 0){
      wheel_speeds[0] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                        + TICK_TRUST * 1000.0f * TICK_LEN / ((tickTime-left_ticks_times[(left_times_index-1)%TICKTIMESSIZE]));
    }
    if(right_ticks_times[(right_times_index-1)%TICKTIMESSIZE] != 0){
      wheel_speeds[1] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                        + TICK_TRUST * 1000.0f * TICK_LEN / ((tickTime-right_ticks_times[(right_times_index-1)%TICKTIMESSIZE]));
    }
*/

void calcSpeeds(double dT){
  static float last_ticks_l = 0;
  static float last_ticks_r = 0;
  Serial.println(left_ticks-last_ticks_l);
  wheel_speeds[0] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                    + TICK_TRUST * (TICK_LEN*(left_ticks - last_ticks_l)/dT);
  last_ticks_l = left_ticks;
    
  wheel_speeds[1] = (1.0f-TICK_TRUST) * wheel_speeds[1] 
                    + TICK_TRUST * (TICK_LEN*(right_ticks - last_ticks_r)/dT);
  last_ticks_r = right_ticks;
    
}

void setMotors(int val1, int val2) {
  setMotor('A', val1);
  setMotor('B', val2);
}

void setMotor(char m, int value) {
  unsigned int abs_value = abs(value);
  switch (m) {
    case 'A':
    case 'a':
      if (value > 0)
      {
        left_direction = 1;
        digitalWrite(D3, HIGH);
      }
      else if (value < 0)
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
      if (value > 0)
      {
        right_direction = 1;
        digitalWrite(D4, LOW);
      }
      else if (value < 0)
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

void speedControl(float dT, float vSoll, float rSoll, int* output) {
  static float integral_part_l = 0;
  static float integral_part_r = 0;
  static float last_error_l = 0;
  static float last_error_r = 0;

  float k_p = 2.0; //3
  float k_d = 1.0; //3 
  float k_i = 0.01; //0.01

  bool switch_sides = rSoll < 0;

  float radius = fabs(rSoll);
  
  float rSollMax = radius+ROBOT_WIDTH_2;
  float rSollMin = radius-ROBOT_WIDTH_2;
  float vInner = vSoll * (rSollMin / radius);
  float vAusser = vSoll * (rSollMax / radius);

  if(abs(rSoll) - ROBOT_WIDTH_2 <= 0.1 ){
    vInner = 0;
    vAusser = 2.0f*vSoll;
  }
  if(abs(rSoll) >= 300){
    vInner = vSoll;
    vAusser = vSoll;
  }
  
  float vLinks = vAusser;
  float vRechts = vInner;
  
  if(switch_sides){
    vLinks = vInner;
    vRechts = vAusser;
  }

  float error_l = vLinks - wheel_speeds[0];
  float error_r = vRechts - wheel_speeds[1];

  float pVal_l = error_l * k_p;
  float dVal_l = (error_l-last_error_l) * k_d;
  integral_part_l += error_l * k_i;
  last_error_l = error_l;

  
  float pVal_r = error_r * k_p;
  float dVal_r = (error_r-last_error_r) * k_d;
  integral_part_r += error_r * k_i;
  last_error_r = error_r;

//  Serial.print("p: ");
//  Serial.print(pVal_l);
//  Serial.print("i: ");
//  Serial.print(integral_part_l);
//  Serial.print("d: ");
//  Serial.print(dVal_l);
//  Serial.println();

  output[0] = (int)(pVal_l - dVal_l + integral_part_l);
  output[1] = (int)(pVal_r - dVal_r + integral_part_r);
}


float left_tps = 0;
float right_tps = 0;
float iVal_left = 0;
float iVal_right = 0;
bool wheelSpeedControl(char wheel, float ticksPerSecond, float dT, float * correction, float clampAroundZero) {
  float proportional = 20.0;
  float differential = 3.0;
  float integral = 0.001;

  if (clampAroundZero < 0) {
    clampAroundZero = -1 * clampAroundZero;
  }

  int wheelTicks = 0;
  float tps_last = 0;
  float iVal = 0;
  long * timesArray;
  int index;
  switch (wheel) {
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

  float tps = 1000.0f / ((float)(timesArray[(index - 1) % TICKTIMESSIZE] - timesArray[(index)]) / (float)TICKTIMESSIZE); //tick_diff/dT;
  float dError = tps - tps_last;
  float error = ticksPerSecond - tps;

  float pVal = proportional * error;
  float dVal = differential * dError;
  iVal += integral * error;

  iVal = max(-70, min(70, iVal));

  *correction = max(-1 * clampAroundZero, min(clampAroundZero, pVal - dVal + iVal));

  switch (wheel) {
    case 'A':
    case 'a':
      iVal_left = iVal;
      left_tps = tps;
//      Serial.print("pid:\t");
//      Serial.println(*correction );
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

void ticksControl(int speed1, int speed2, int* output) {

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
  digitalWrite(D3, HIGH);
  digitalWrite(D4, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  attachInterrupt(D5, tick_left_ISR, CHANGE);
  attachInterrupt(D6, tick_right_ISR, CHANGE);

  Serial.begin(115200);
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  motor_speeds[0] = 950;
  motor_speeds[1] = 950;
  stoptime = millis();
  stoptime2 = stoptime;
  Serial.println("START");
  setMotors(0, 0);
  motor_speeds[0] = 0;
  motor_speeds[1] = 0;
  delay(100);
}

int kurvenradius = -40.0f;
bool forward = true;
void loop() {
  //
  //  if(millis() - stoptime > 20)
  //  {
  //    ticksControl(650, 650, motor_speeds);
  //    setMotors(motor_speeds[0], motor_speeds[1]);
  //    stoptime+=20;
  //  }

//  if (millis() - stoptime2 > 150)
//  {
//    int speeds = 750;
//    float * cVal = new float;
//    if (wheelSpeedControl('A', 50, 150.0f / 1000.0f, cVal, 200)) {
//      setMotor('A', (int)(speeds + (*cVal)));
//    }
//    if (wheelSpeedControl('B', 50, 150.0f / 1000.0f, cVal, 200)) {
//      setMotor('B', (int)(speeds + (*cVal)));
//    }
//    delete cVal;
//    stoptime += 150;
//  }

  if (millis() - stoptime2 > 150)
  {
    int speeds = 750;
    calcSpeeds(0.150f);
    int * cVal = new int[2];
    speedControl(1.0f, 20.0f, kurvenradius, cVal);
    motor_speeds[0] = max(0, min(motor_speeds[0]+cVal[0], 1024));
    motor_speeds[1] = max(0, min(motor_speeds[1]+cVal[1], 1024));
    delete [] cVal;
    setMotors(motor_speeds[0], motor_speeds[1]);
    stoptime2 += 150;
  }

  if(millis() - stoptime > 10000){
    stoptime = millis();
    kurvenradius *= -1;
  }

//    for(int i = 0; i<TICKTIMESSIZE; i++){
//      Serial.println(left_ticks_times[(left_times_index + i)%TICKTIMESSIZE]);
//    }
//    Serial.println("\n\n");

}
