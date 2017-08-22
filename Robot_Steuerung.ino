#include <ESP8266WiFi.h>
#include <Arduino.h>
#include "scheduling.h"

#define TICKTIMESSIZE 10

//Alle Abmessungen in cm
#define WHEEL_DIAMETER 7.1f
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0f
//#define WHEEL_CIRC  WHEEL_DIAMETER * PI
#define WHEEL_CIRC  21.4f
#define TICK_LEN  WHEEL_CIRC/96.0f

#define ROBOT_WIDTH 13.5f
#define ROBOT_WIDTH_2 ROBOT_WIDTH/2.0f

#define TICK_TRUST 0.3f

#define RINGBUFFERSIZE 128

//#define NOMOTORS
//ESP.getFreeHeap() // freier Arbeitsspeicher ##########>>> NICHT löschen, das brauchst du sowieso wieder! <<<########## 

const String commands[] = {"SendStatus", "Ok", "TurnOn"};

const char *ssid = "CAR-WLAN";

Scheduler task_scheduler;

IPAddress localip(192, 168, 178, 35);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);
int port = 5000;

WiFiServer server(port);

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
int motor_setSpeeds[2];

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

int clamp(int a, int minval, int maxval){
  if(a<minval) return minval;
  if(a>maxval) return maxval;
  return a;
}

void calcSpeeds(double dT){
  static float last_ticks_l = 0;
  static float last_ticks_r = 0;
  wheel_speeds[0] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                    + TICK_TRUST * (TICK_LEN*(left_ticks - last_ticks_l)/dT);
  last_ticks_l = left_ticks;
    
  wheel_speeds[1] = (1.0f-TICK_TRUST) * wheel_speeds[1] 
                    + TICK_TRUST * (TICK_LEN*(right_ticks - last_ticks_r)/dT);
  last_ticks_r = right_ticks;
    
}

int sign(int num){
  return num>=0?1:-1;
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
      motor_speeds[0] = value;
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
      motor_speeds[1] = value;
      break;
    default:
      break;
  }
}

void setMotors(int val1, int val2) {
//#ifdef NOMOTORS
//  setMotor('A', 0);
//  setMotor('B', 0);
//#else
  setMotor('A', val1);
  setMotor('B', val2);
//#endif
}

void speedControl(float dT, float vSoll, float rSoll, int* output) {
  static float integral_part_l = 0;
  static float integral_part_r = 0;
  static float last_error_l = 0;
  static float last_error_r = 0;

  float k_p = 10.0; //3
  float k_d = 10.0; //3 
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

  integral_part_l = clamp(integral_part_l, -200, 200);
  integral_part_r = clamp(integral_part_r, -200, 200);

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

  iVal = clamp(iVal, -70, 70);

  *correction = clamp( pVal - dVal + iVal,-1 * clampAroundZero,clampAroundZero);

  switch (wheel) {
    case 'A':
    case 'a':
      iVal_left = iVal;
      left_tps = tps;
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

  float control_output = clamp(proportional_tickcontrol - differential_tickcontrol + integral_tickcontrol, -200, 200);

  output[0] = clamp(speed1 - control_output, 0, 1024);
  output[1] = clamp(speed2 - control_output, 0, 1024);

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

//Ab hier: WIFI
void wifiSetup() {
//  WiFi.mode(WIFI_AP_STA);
//  WiFi.softAPConfig(localip, gateway, subnet);
//  WiFi.softAP(ssid);
//  IPAddress myIP = WiFi.softAPIP();
  WiFi.mode(WIFI_STA);
  //WiFi.config(localip, gateway, subnet);
  WiFi.begin("Ultron", "Zu120%SICHERundMITsicherMEINichSICHER");
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed");
    while (true) {}
  }

  Serial.println("Connected.");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("IP:  ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.print("Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("Status: ");
  Serial.println(WiFi.status());
  server.begin();
}

//Ab hier: Tasks

WiFiClient working_client;
float kurvenradius = 0.0f;
float geschwindigkeit = 0.0f;
bool forward = true;

uint8_t * data_ring_buffer = new uint8_t[RINGBUFFERSIZE]();
int ring_buffer_write_position = 0;
int ring_buffer_read_position = 0;

int bufferFramePosition(){
  int start_index = -1;
  int end_index = -1;
  for(int i = ring_buffer_read_position; i!=ring_buffer_write_position; i=(i+1)%RINGBUFFERSIZE){
    if((char)data_ring_buffer[i] == '{'){
      start_index = i;
    }
    if((char)data_ring_buffer[i] == '}'){
      end_index = i;
      if(start_index >= 0){
        return start_index;
      }
    }
  }
  return -1;
}

bool bufferGetFrame(uint8_t ** frame, int ** frame_size){
  int start_index = bufferFramePosition();
  if(start_index >= 0){
    ring_buffer_read_position = start_index%RINGBUFFERSIZE;
  } else {
    return false;
  }
  int end_index = -1;
  for(int i = ring_buffer_read_position; i!=ring_buffer_write_position; i=(i+1)%RINGBUFFERSIZE){
    if((char)data_ring_buffer[i] == '}'){
      end_index = i;
      break;
    }
  }
  int frame_len = end_index-start_index+1;
  if(frame_len < 0){
    frame_len = RINGBUFFERSIZE - start_index + end_index +1;
  }
  *frame = (uint8_t*) malloc((frame_len)*sizeof(uint8_t));
  *frame_size = (int*) malloc(sizeof(int));
  **frame_size = frame_len;
  int write_pos = 0;
  for(int i = start_index; i!=(end_index+1)%RINGBUFFERSIZE; i=(i+1)%RINGBUFFERSIZE){
    (*frame)[write_pos] = data_ring_buffer[i];
    write_pos++;
  }
  ring_buffer_read_position = (end_index+1)%RINGBUFFERSIZE;
  return true;
}

bool writeToBuffer(uint8_t* input, int input_size){
  int pos = 0;
  while(pos < input_size){
    data_ring_buffer[ring_buffer_write_position] = input[pos];
    if(ring_buffer_write_position == ring_buffer_read_position){
      ring_buffer_read_position = (ring_buffer_read_position+1)%RINGBUFFERSIZE;
    }
    ring_buffer_write_position = (ring_buffer_write_position+1)%RINGBUFFERSIZE;
    pos += 1;
  }
  if(ring_buffer_write_position == ring_buffer_read_position){
    ring_buffer_read_position += 1;
    ring_buffer_read_position = ring_buffer_read_position%RINGBUFFERSIZE;
  }
}

void frameToInstructions(uint8_t * frame, int frame_size){
  String * instructions = new String[2]();
  instructions[0] = String("");
  instructions[1] = String("");
  int pos = 0;
  while(frame[pos] != '{' && pos < frame_size){
    pos+=1;
  }
  pos+=1;
  while(frame[pos] != ',' && pos < frame_size){
    instructions[0] += (char)frame[pos];
    pos+=1;
  }
  pos+=1;
  while(frame[pos] != '}' && pos < frame_size){
    instructions[1] += (char)frame[pos];
    pos+=1;
  }
  geschwindigkeit = instructions[0].toFloat();//0;//
  kurvenradius = instructions[1].toFloat();//0;//
//  Serial.print("G: ");
//  Serial.print(geschwindigkeit);
//  Serial.print(".");
//  Serial.print("R: ");
//  Serial.println(kurvenradius);
  delete [] instructions;
}

int comReadoutTask(){
  uint8_t * frame = NULL;
  int * frame_size = NULL;
  if(bufferGetFrame(&frame, &frame_size)){
    frameToInstructions(frame, *frame_size);
    free(frame);
    free(frame_size);
  }
  return 0;
}

int comTask(){
  
  if(!working_client || !working_client.connected()){
    working_client = server.available();
  }
  
  if(working_client.connected()){
    if(working_client.available()){
      int available_bytes = working_client.available();
      uint8_t * raw = (uint8_t*)malloc((available_bytes)*sizeof(uint8_t));
      working_client.read(raw, available_bytes);
      writeToBuffer(raw, available_bytes);
      free(raw);
      working_client.println(".");
      working_client.flush();
//      Serial.println(ESP.getFreeHeap());
//      for(int i = 0; i <= RINGBUFFERSIZE; i++){
//        Serial.print((char)data_ring_buffer[i]);
//      }
//      Serial.println();
    }
  }
  return 0;
}

int motorControlTask(){
    calcSpeeds(0.150f);
    int * cVal = new int[2];
    speedControl(1.0f, geschwindigkeit, kurvenradius, cVal);
    motor_setSpeeds[0] = clamp(motor_setSpeeds[0]+cVal[0], 0, 1024);
    motor_setSpeeds[1] = clamp(motor_setSpeeds[1]+cVal[1], 0, 1024);
    delete [] cVal;
    setMotors(motor_setSpeeds[0], motor_setSpeeds[1]);
    stoptime2 += 150;
    return 0;
}

//Ab hier: Setup und main Loop

void setup() {
  //PIN SETUP
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

  //COM SETUP
  Serial.begin(115200);
  wifiSetup();

  //TASKS SETUP
  stoptime = millis();
  String motorctrlTaskName = "motor-ctrl";
  task_scheduler.addFunction(&motorControlTask, motorctrlTaskName, 150, millis(), 10, 0, 0);
  String comTaskName = "communication";
  task_scheduler.addFunction(&comTask, comTaskName, 1, millis(), 10, 0, 0);
  String comReadoutTaskName = "comReadout";
  task_scheduler.addFunction(&comReadoutTask, comReadoutTaskName, 1, millis(), 7, 0, 0);

  //DEFAULTS EINSTELLEN
  setMotors(0, 0);
  motor_speeds[0] = 0;
  motor_speeds[1] = 0;
  motor_setSpeeds[0] = 0;
  motor_setSpeeds[1] = 0;
  
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.println("START");
}

void loop() {
  task_scheduler.execute();
  delay(1);

}
