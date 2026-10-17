const int key1Pin = 2;
const int key2Pin = 3;
const int key3Pin = 4;
const int key4Pin = 5;

// 按键状态变量
int key1Last = HIGH;
int key2Last = HIGH;
int key3Last = HIGH;
int key4Last = HIGH;
unsigned long debounceDelay = 50; //消抖时间ms

void setup() {
  Serial.begin(9600); 
  pinMode(key1Pin, INPUT_PULLUP);
  pinMode(key2Pin, INPUT_PULLUP);
  pinMode(key3Pin, INPUT_PULLUP);
  pinMode(key4Pin, INPUT_PULLUP);
}

void loop() {
  readAndSendKey(key1Pin, key1Last, 'K');
  readAndSendKey(key2Pin, key2Last, 'T');
  readAndSendKey(key3Pin, key3Last, 'P');
  readAndSendKey(key4Pin, key4Last, 'i');
}

// 按键读取+消抖+发送指令函数
void readAndSendKey(int pin, int &lastState, char cmdChar)
{
  int nowState = digitalRead(pin);
  if(nowState != lastState)
  {
    delay(debounceDelay); //消抖
    nowState = digitalRead(pin);
    if(nowState != lastState)
    {
      // 检测到按下
      if(nowState == LOW)
      {
        Serial.write(cmdChar); //串口发送单个字符指令
      }
      lastState = nowState;
    }
  }
}