int left_ticks = 0;
int left_direction = 1;

int right_ticks = 0;
int right_direction = 1;

long stoptime = 0;
bool driving_status = true;

void tick_left_ISR() {
  left_ticks += left_direction;
}

void tick_right_ISR() {
  right_ticks += right_direction;
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
  stoptime = millis();
  Serial.println("START");
}

 int relative_position = 48;
 bool forward = true;
void loop() {

    Serial.print("ticks (l,r): ");
    Serial.print(left_ticks);
    Serial.print(", ");
    Serial.println(right_ticks);
  

  if (driving_status)
  {
    switch(left_direction){
      case 1:
          digitalWrite(D3, HIGH);
          analogWrite(D1, 950);
          break;
      case -1:
          digitalWrite(D3, LOW);
          analogWrite(D1, 950);
          break;
      default:
          digitalWrite(D3, LOW);
          analogWrite(D1, 0);
          break;
    }
    switch(right_direction){
      case 1:
          digitalWrite(D4, LOW);
          analogWrite(D2, 950);
          break;
      case -1:
          digitalWrite(D4, HIGH);
          analogWrite(D2, 950);
          break;
      default:
          digitalWrite(D4, LOW);
          analogWrite(D2, 0);
          break;
    }
  }
  else
  {
    digitalWrite(D3, LOW);
    digitalWrite(D4, LOW);
    analogWrite(D1, 0);
    analogWrite(D2, 0);
  }

  if(left_direction != 0 || right_direction != 0){
    if(left_ticks >= relative_position && forward)
    {
      left_direction = 0;
    }
    if(left_ticks < 0 && !forward)
    {
      left_direction = 0;
    }
    
    if(right_ticks >= relative_position && forward)
    {
      right_direction = 0;
    }
    if(right_ticks < 0 && !forward)
    {
      right_direction = 0;
    }
  }
  else
  {
    if(forward){
      forward = false;
      left_direction = -1;
      right_direction = -1;
    }
    else
    {
      forward = true;
      left_direction = 1;
      right_direction = 1;
    }
  }
  
}
