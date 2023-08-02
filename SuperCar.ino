
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define AIN1 26
#define AIN2 27
#define BIN1 17
#define BIN2 5

#define PWM1  0
#define PWM2  1
#define PWM3  2
#define PWM4  3
#define freq 50000      //PWM波形频率5KHZ
#define resolution  10    //使用PWM占空比的分辨率，占空比最大可写2^10-1=1023

#define RA  12
#define RB  14
#define LA  18
#define LB  19

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
uint8_t txValue = 0;                         //后面需要发送的值
BLEServer *pServer = NULL;                   //BLEServer指针 pServer
BLECharacteristic *pTxCharacteristic = NULL; //BLECharacteristic指针 pTxCharacteristic
bool deviceConnected = false;                //本次连接状态
BLECharacteristic controlCharacteristic(SERVICE_UUID, 
                                        BLECharacteristic::PROPERTY_READ | 
                                        BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_NOTIFY);

long cnt_L, cnt_R = 0;
int Velocity_Left, Velocity_Right = 0;
int pwm_v = 650, speed_step = 5; //Motor pwm speed
hw_timer_t *timer = NULL;

void Left_encoder_isr_A();
void Left_encoder_isr_B();
void front(int pwm_v);
void left (int pwm_v);
void right(int pwm_v);
void back (int pwm_v);
void sstop();
//void IRAM_ATTR TimerEvent();
void IRAM_ATTR update_speed();


class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        Serial.println("SuperCar Connected.");
        deviceConnected = true;
    };
 
    void onDisconnect(BLEServer *pServer)
    {
        Serial.println("SuperCar Disonnected.");
        deviceConnected = false;
    }
};
 
class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue(); //接收信息
 
        if (rxValue.length() > 0)
        { //向串口输出收到的值
            Serial.print("RX: ");
            for (int i = 0; i < rxValue.length(); i++)
                Serial.print(rxValue[i]);
            Serial.println();
        }
    }
};


void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RB), Left_encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB), Left_encoder_isr_B, CHANGE);

  // PWM settings  
  // Wheel 1 forward
  ledcSetup(PWM1, freq, resolution);  //PWM通道一开启设置
  ledcAttachPin(AIN1, PWM1);     //PWM通道一和引脚PWMA关联
  ledcWrite(PWM1, 0);        //PWM通道一占空比设置为零
  // Wheel 1 backward
  ledcSetup(PWM2, freq, resolution);
  ledcAttachPin(AIN2, PWM2); 
  ledcWrite(PWM2, 0); 

  // PWM settings  
  // Wheel 2 forward
  ledcSetup(PWM3, freq, resolution);
  ledcAttachPin(BIN1, PWM3); 
  ledcWrite(PWM3, 0); 
  // Wheel 2 backward
  ledcSetup(PWM4, freq, resolution);
  ledcAttachPin(BIN2, PWM4);
  ledcWrite(PWM4, 0);
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &update_speed, true);
  timerAlarmWrite(timer, 100000, true); //100ms update
  timerAlarmEnable(timer);

  BLEDevice::init("SuperCar");//在这里面是ble的名称
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pServer->setCallbacks(new MyServerCallbacks());

  pService->addCharacteristic(&controlCharacteristic);
  controlCharacteristic.setValue("Control");
  pService->start();                  // 开始服务
  pServer->getAdvertising()->start(); // 开始广播
  Serial.println(" Super Car waiting for connection... ");
}

char cmd = 's';
void loop() {
    
  Serial.print("Voltage: ");
  Serial.print(pwm_v);
  Serial.print(" VR: ");
  Serial.print(Velocity_Left);
  Serial.print(" VL: ");
  Serial.println(Velocity_Right);

  if(Serial.available())
    cmd = (char)Serial.read();

  if(deviceConnected){
    std::string recv = controlCharacteristic.getValue();
    cmd = recv[0];
  }

  switch(cmd){
    case 'w': 
      if(deviceConnected) {controlCharacteristic.setValue("front"); controlCharacteristic.notify();}
      front(pwm_v);
      break;
    case 'a':
      if(deviceConnected) controlCharacteristic.setValue("left");
      left(pwm_v);
      break;
    case 'd':
      if(deviceConnected) controlCharacteristic.setValue("right");
      right(pwm_v);
      break;
    case 'x':
      if(deviceConnected) controlCharacteristic.setValue("back");
      back(pwm_v);
      break;
    case 's':
      if(deviceConnected) controlCharacteristic.setValue("stop");
      sstop();
      break;
    case '+':
      if(deviceConnected) controlCharacteristic.setValue("speed up");
      if(pwm_v+100<1000) pwm_v+=speed_step;
      break;
    case '-':
      if(deviceConnected) controlCharacteristic.setValue("speed down");
      if(pwm_v-600>0   ) pwm_v-=speed_step;
      break;
    default:
      ;
  }
  cmd = 's';

}

void IRAM_ATTR update_speed(){
  Velocity_Left = cnt_L;
  cnt_L = 0;
  Velocity_Right = cnt_R;
  cnt_R = 0;

}

void Left_encoder_isr_A(){
  if (digitalRead(RA)!=digitalRead(RB)) cnt_L++;
  else cnt_L--;
}

void Left_encoder_isr_B(){
  if (digitalRead(LA)!=digitalRead(LB)) cnt_R--;
  else  cnt_R++; 
}

void front(int pwm_v){
    ledcWrite(PWM1, 0);
    ledcWrite(PWM2, pwm_v);
    ledcWrite(PWM3, pwm_v);
    ledcWrite(PWM4, 0);
    delay(20);
}

void back(int pwm_v){
    ledcWrite(PWM1, pwm_v);
    ledcWrite(PWM3, 0);
    ledcWrite(PWM2, 0);
    ledcWrite(PWM4, pwm_v);
    delay(20);
}

void left(int pwm_v){
    ledcWrite(PWM1, 0);
    ledcWrite(PWM2, pwm_v); //rigth forward
    ledcWrite(PWM3, 0);
    ledcWrite(PWM4, pwm_v);
    delay(20);
}

void right(int pwm_v){
    ledcWrite(PWM1, pwm_v);
    ledcWrite(PWM2, 0);
    ledcWrite(PWM3, pwm_v);
    ledcWrite(PWM4, 0);
    delay(20);
}

void sstop(){
    ledcWrite(PWM1, 0);
    ledcWrite(PWM2, 0);
    ledcWrite(PWM3, 0);
    ledcWrite(PWM4, 0);
}
