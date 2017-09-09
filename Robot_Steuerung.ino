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

#define TICK_TRUST 0.1f

#define RINGBUFFERSIZE 128

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

int sign(int num){
  return num>=0?1:-1;
}

int sign(float num){
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

//Ab hier: Tasks

WiFiClient * working_client = new WiFiClient();
float kurvenradius = 30.0f;
float geschwindigkeit = 10.0f;
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
  Serial.print("G: ");
  Serial.print(geschwindigkeit);
  Serial.print(".");
  Serial.print("R: ");
  Serial.println(kurvenradius);
  delete [] instructions;
}

//Feedback
int calcSpeeds(){
  double * dT = (double *) task_scheduler.functionalMem[0];
  double dT_sec = (*dT)/1000.0;
  static int last_ticks_l = 0;
  static int last_ticks_r = 0;
  float dTicks_l = (left_ticks - last_ticks_l) * sign(motor_speeds[0]);
  float dTicks_r = (right_ticks - last_ticks_r) * sign(motor_speeds[1]);
  wheel_speeds[0] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                    + TICK_TRUST * (TICK_LEN*dTicks_l/dT_sec);
  last_ticks_l = left_ticks;
    
  wheel_speeds[1] = (1.0f-TICK_TRUST) * wheel_speeds[1] 
                    + TICK_TRUST * (TICK_LEN*dTicks_r/dT_sec);
  last_ticks_r = right_ticks;

  Serial.print(wheel_speeds[0], DEC);
  Serial.print(" - ");
  Serial.println(wheel_speeds[1], DEC);

  return 0;
}

//Regelung
void speedControl(float dT, float vSoll, float rSoll, int* output) {
  static float integral_part_l = 0;
  static float integral_part_r = 0;
  static float last_speed_l = 0;
  static float last_speed_r = 0;

  float k_p = 20.0;//20.0; //3
  float k_d = 0.0;
  float k_i = 1.0;//1.0;//10.0

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
  float dError_l = wheel_speeds[0] - last_speed_l;
  float error_r = vRechts - wheel_speeds[1];
  float dError_r = wheel_speeds[1] - last_speed_r;

  float pVal_l = vLinks * k_p;
  float dVal_l = dError_l * k_d;
  integral_part_l += error_l * k_i;
  last_speed_l = wheel_speeds[0];

  
  float pVal_r = vRechts * k_p;
  float dVal_r = dError_r * k_d;
  integral_part_r += error_r * k_i;
  last_speed_r = wheel_speeds[1];

  output[0] = (int)(pVal_l - dVal_l + integral_part_l);
  output[1] = (int)(pVal_r - dVal_r + integral_part_r);
}

int comReadoutTask(){
  if(working_client && working_client->connected()){
    uint8_t * frame = NULL;
    int * frame_size = NULL;
    if(bufferGetFrame(&frame, &frame_size)){
      frameToInstructions(frame, *frame_size);
      free(frame);
      free(frame_size);
    }
  }
  else{
    if(!working_client || !working_client->connected()){
      kurvenradius = 0;
      geschwindigkeit = 0;
      setMotors(0,0);
    }
  }
  return 0;
}

//Crasht bei niedrigem Batteriestand
int comTask(){
//  Serial.println("1");
  if(!working_client || !working_client->connected()){
    *working_client = server.available();
  }
//  Serial.println("2");
  if(working_client->connected()){
//    Serial.println("3");
    if(working_client->available()){
//      Serial.println("4");
      int available_bytes = working_client->available();
      uint8_t * raw = (uint8_t*)malloc((available_bytes)*sizeof(uint8_t));
      working_client->read(raw, available_bytes);
//      Serial.println("5");
      writeToBuffer(raw, available_bytes);
      free(raw);
//      Serial.println("6");
      working_client->println(".");
      //working_client->flush();
//      Serial.println(ESP.getFreeHeap());
//      for(int i = 0; i <= RINGBUFFERSIZE; i++){
//        Serial.print((char)data_ring_buffer[i]);
//      }
//      Serial.println();
//      Serial.println("7");
    }
  }
  return 0;
}

int motorControlTask(){
  
    double * dT = (double *) task_scheduler.functionalMem[0];
    double dT_sec = (*dT)/1000.0;
  
    static int * cVal = new int[2];
    static int * cVal_last = new int[2];
    cVal_last[0] = cVal[0];
    cVal_last[1] = cVal[1];
    speedControl(dT_sec, geschwindigkeit, kurvenradius, cVal);

    //Restrict acceleration
    int * d_cVal = new int[2];
    d_cVal[0] = cVal[0] - cVal_last[0];
    d_cVal[1] = cVal[1] - cVal_last[1];
//    d_cVal[0] = clamp(d_cVal[0], -50, 50);
//    d_cVal[1] = clamp(d_cVal[1], -50, 50);
    
    cVal[0] = cVal_last[0] + d_cVal[0];
    cVal[1] = cVal_last[1] + d_cVal[1];
    motor_setSpeeds[0] = clamp(cVal[0], 0, 1024); //motor_setSpeeds[0]+
    motor_setSpeeds[1] = clamp(cVal[1], 0, 1024); //motor_setSpeeds[1]+
    delete [] d_cVal;

    if(geschwindigkeit == 0){
      motor_setSpeeds[0] = 0;
      motor_setSpeeds[1] = 0;
    }
    setMotors(motor_setSpeeds[0], motor_setSpeeds[1]);
//    Serial.print("(");
//    Serial.print(motor_setSpeeds[0], DEC);
//    Serial.print(",");
//    Serial.print(motor_setSpeeds[1], DEC);
//    Serial.println(")");
//    Serial.println(ESP.getFreeHeap());
    return 0;
}

//Ab hier: WIFI
void wifiSetup() {
  Serial.println("Setting up WiFI");
//  WiFi.mode(WIFI_AP_STA);
//  WiFi.softAPConfig(localip, gateway, subnet);
//  WiFi.softAP(ssid);
//  IPAddress myIP = WiFi.softAPIP();
  WiFi.mode(WIFI_STA);
  //WiFi.config(localip, gateway, subnet);
  WiFi.begin("Ultron", "Zu120%SICHERundMITsicherMEINichSICHER");
  delay(1000);
//  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Connection Failed");
//    while (true) {
//      Serial.println("Connection Failed");
//      }
//  }

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

  //Attach Tasks
  String comTaskName = "communication";
  task_scheduler.addFunction(&comTask, comTaskName, 10, millis(), 5, 0, 0);
  String comReadoutTaskName = "comReadout";
  task_scheduler.addFunction(&comReadoutTask, comReadoutTaskName, 1, millis(), 7, 0, 0);
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

  //TASKS SETUP
  wifiSetup();
  String motorctrlTaskName = "motor-ctrl";
  double * arg_0 = new double();
  *arg_0 = 150.0;
  int ** args_0 = new int*[1];
  args_0[0] = (int*)arg_0;
  task_scheduler.addFunction(&motorControlTask, motorctrlTaskName, *arg_0, millis(), 10, args_0, 1);

  
  String speedUpdateTaskName = "motor-speed";
  double * arg_1 = new double();
  *arg_1 = 30.0;
  int ** args_1 = new int*[1];
  args_1[0] = (int*)arg_1;
  task_scheduler.addFunction(&calcSpeeds, speedUpdateTaskName, *arg_1, millis(), 50, args_1, 1);
}

void loop() {
  task_scheduler.execute();
}
