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
void loop() {

    Serial.print("ticks (l,r): ");
    Serial.print(left_ticks);
    Serial.print(", ");
    Serial.println(right_ticks);
  

  if (driving_status)
  {
    digitalWrite(D3, HIGH);
    digitalWrite(D4, LOW);
    analogWrite(D1, 950);
    analogWrite(D2, 950);
  }
  else
  {
    digitalWrite(D3, LOW);
    digitalWrite(D4, LOW);
    analogWrite(D1, 0);
    analogWrite(D2, 0);
  }

  if(left_ticks < relative_position && right_ticks < relative_position)
  {
    driving_status = true;
  }
  else 
  {
    driving_status = false;
  }

  if (millis() - stoptime >= 5000)
  {
    while(left_ticks >= relative_position) left_ticks -= relative_position;
    while(right_ticks >= relative_position) right_ticks -= relative_position;
    stoptime += 5000;
  }
  
}
