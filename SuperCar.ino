#include <Arduino.h>
#define AIN1 26
#define AIN2 27
#define BIN1 17
#define BIN2 5

#define RA  12
#define RB  14
#define LA  18
#define LB  19
#define MOVING_PULSE 100

long cnt_L, cnt_R = 0;
int Velocity_Left, Velocity_Right = 0;
hw_timer_t *timer = NULL;

void Left_encoder_isr_A();
void Left_encoder_isr_B();
void front(int ratio);
void left (int ratio);
void right(int ratio);
void back (int ratio);
void sstop();
//void IRAM_ATTR TimerEvent();
void IRAM_ATTR update_speed();

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RB), Left_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB), Left_encoder_isr_B, CHANGE);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &update_speed, true);
  timerAlarmWrite(timer, 100000, true); //100ms update
  timerAlarmEnable(timer);
}

void loop() {
//  Serial.print("R: ");
//  Serial.print(cnt_R);
//  Serial.print(" L: ");
//  Serial.println(cnt_L);

  Serial.print("VR: ");
  Serial.print(Velocity_Left);
  Serial.print(" VL: ");
  Serial.println(Velocity_Right);

//  front(10);
}

void IRAM_ATTR update_speed(){
  Velocity_Left = cnt_L;
  cnt_L = 0;
  Velocity_Right = cnt_R;
  cnt_R = 0;

}

void IRAM_ATTR TimerEvent()
{
    Serial.println("Timer called");
}

void Left_encoder_isr_A(){
  if (digitalRead(RA)!=digitalRead(RB)) cnt_L++;
  else cnt_L--;
}

void Left_encoder_isr_B(){
  if (digitalRead(LA)!=digitalRead(LB)) cnt_R++;
  else  cnt_R--; 
}

void front(int ratio){
  Serial.println("FRONT");
  if(ratio > 100) ratio%=100;
  int empty = MOVING_PULSE - ratio;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  delay(ratio);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(empty);
}

void left(int ratio){
  Serial.println("LEFT");
  if(ratio > 100) ratio%=100;
  int empty = MOVING_PULSE - ratio;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  delay(ratio);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(empty);
}
void right(int ratio){
  Serial.println("RIGHT");
  if(ratio > 100) ratio%=100;
  int empty = MOVING_PULSE - ratio;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  delay(ratio);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(empty);
}
void back(int ratio){
  Serial.println("BACK");
  if(ratio > 100) ratio%=100;
  int empty = MOVING_PULSE - ratio;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  delay(ratio);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(empty);
}
void sstop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
