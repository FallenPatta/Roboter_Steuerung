const int pin = 13;

void toggle() {
  static int state = 0;
  state = !state;
  digitalWrite(BUILTIN_LED, state);
}

void setup() {
  //pinMode(BUILTIN_LED, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, LOW);
  digitalWrite(BUILTIN_LED, LOW);
    for(int i = 0; i<10; i++){
      digitalWrite(BUILTIN_LED, HIGH);
      delay(100);
      digitalWrite(BUILTIN_LED, LOW);
      delay(100);
    }
    Serial.begin(115200);
    Serial.println("START");
    Serial.println(D1);
    Serial.println(D2);
    Serial.println(D3);
    Serial.println(D4);
    Serial.println(D5);
    Serial.println(D6);
//    Serial.println(BUILTIN_LED);
//  pinMode(pin, INPUT);
//  attachInterrupt(pin, toggle, CHANGE);
//  toggle();
}

bool last_left = 0;
int left_ticks = 0;
long stoptime = 0;
bool driving_status = true;
void loop() {
  if(last_left != digitalRead(D5)){
    last_left = !last_left;
    left_ticks++;
    Serial.println(left_ticks); 
  }
  if(left_ticks < 200){
    digitalWrite(D3, HIGH);
    digitalWrite(D4, LOW);
    analogWrite(D1, 1024);
    analogWrite(D2, 1024);
    driving_status = true;
  } else if (driving_status) {
    digitalWrite(D3, LOW);
    digitalWrite(D4, LOW);
    analogWrite(D1, 0);
    analogWrite(D2, 0);
    stoptime = millis();
    driving_status = false;
  }
    
  if(!driving_status && millis() - stoptime >= 10000){
    left_ticks = 0;
  }
//  digitalWrite(D3, HIGH);
//  digitalWrite(D4, LOW);
//    analogWrite(D1, 900);
//    analogWrite(D2, 900);
//    delay(250);
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, LOW);
//    analogWrite(D1, 0);
//    analogWrite(D2, 0);
//    delay(250);
//    
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, HIGH);
//    analogWrite(D1, 900);
//    analogWrite(D2, 900);
//    delay(250);
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, LOW);
//    analogWrite(D1, 0);
//    analogWrite(D2, 0);
//    delay(250);
//    
//      digitalWrite(D3, HIGH);
//  digitalWrite(D4, HIGH);
//    analogWrite(D1, 900);
//    analogWrite(D2, 900);
//    delay(250);
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, LOW);
//    analogWrite(D1, 0);
//    analogWrite(D2, 0);
//    delay(250);
//    
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, LOW);
//    analogWrite(D1, 900);
//    analogWrite(D2, 900);
//    delay(250);
//    digitalWrite(D3, LOW);
//    digitalWrite(D4, LOW);
//    analogWrite(D1, 0);
//    analogWrite(D2, 0);
//    delay(10000);
  }
