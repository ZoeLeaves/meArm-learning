#include <Servo.h>                //使用servo库
Servo base, larm, rarm, claw ;    //创建4个servo对象

// 单个动作点：存储4个舵机的角度
struct ActionPoint { 
  int angbase;
  int angrarm;
  int anglarm;
  int angclaw;
};

//存储电机极限值
const int basemin = 0;
const int basemax = 180;
const int rarmmin = 20;
const int rarmmax = 115;
const int larmmin = 90;
const int larmmax = 145;
const int clawmin = 81;
const int clawmax = 178;
const int maxActPoints = 100; // 最大录制动作点数
const int l1,l2,baseoffset,rarmoffset,larmoffset;
int dsd = 15; //运动延迟时间，用于控制电机运行速度
int step = 1; //控制摇杆移动步长
int samplebreak = 200; // 采样间隔（毫秒），越小动作越流畅
int actionCnt = 0; //已录制动作点数
unsigned long lastsample = 0; // 上一次采样的时间
ActionPoint actionBuf[maxActPoints]; // 动作缓冲区

bool mode = 1; //1为指令模式，0待定
bool isRecording = false; // 录制状态标志
bool isPlaying = false; // 回放状态标志



void setup() {
  //获取引脚数据
  base.attach(9);     
  delay(200);          
  larm.attach(8);     
  delay(200);          
  rarm.attach(7);      
  delay(200);          
  claw.attach(6);      
  delay(200);          

  
  //初始化舵机位置
  base.write(90); 
  delay(10);
  larm.write(20); 
  delay(10);
  rarm.write(90); 
  delay(10);
  claw.write(81);  
  delay(10); 
  
  Serial.begin(9600);
  Serial.println("Robot Arm Set Up");
}

void loop() {
  
  if (Serial.available()>0) {  
    char serialcmd = Serial.read();
    if ( mode == 1 ) armdatacmd(serialcmd); //指令模式
  }

  //摇杆控制
  if (analogRead(A0) > 800) {
    if (base.read() > 0) {
      base.write(constrain(base.read() - step, basemin, basemax));
      delay(10);

    }

  } else if (analogRead(A0) < 200) {
    if (base.read() < 180) {
      base.write(constrain(base.read() + step, basemin, basemax));
      delay(10);

    }
  }
  if (analogRead(A1) < 200) {
    if (larm.read() > 0) {
      larm.write(constrain(larm.read() - step, larmmin, larmmax));
      delay(10);

    }

  } else if (analogRead(A1) > 800) {
    if (larm.read() < 180) {
      larm.write(constrain(larm.read() + step, larmmin, larmmax));
      delay(10);

    }
  }
  if (analogRead(A2) > 800) {
    if (claw.read() > 0) {
      claw.write(constrain(claw.read() - step, clawmin, clawmax));
      delay(10);

    }

  } else if (analogRead(A2) < 200) {
    if (claw.read() < 180) {
      claw.write(constrain(claw.read() + step, clawmin, clawmax));
      delay(10);

    }
  }
  if (analogRead(A3) > 800) {
    if (rarm.read() > 0) {
      rarm.write(constrain(rarm.read() - step, rarmmin, rarmmax));
      delay(10);

    }

  } else if (analogRead(A3) < 200) {
    if (rarm.read() < 180) {
      rarm.write(constrain(rarm.read() + step, rarmmin, rarmmax));
      delay(10);

    }
  }

  //录制函数调用
  recordaction();

}

//指令模式
void armdatacmd(char serialcmd) {
  if (serialcmd == 'b' || serialcmd == 'c' || serialcmd == 'l' || serialcmd == 'r') {
    int servodata = Serial.parseInt();
    servocmd(serialcmd, servodata, dsd);  // 机械臂舵机运行函数（参数：舵机名，目标角度，速度）
  } 
  else if (serialcmd == 'x') {
    String buf = Serial.readStringUntil('\n');
    int x,y,z;
    if(sscanf(buf.c_str(), "%d,y%d,z%d", &x,&y,&z) == 3){
      servocmd('b', x, dsd);
      servocmd('l', y, dsd);
      servocmd('r', z, dsd);
    }
  }
  else {
    switch(serialcmd) {   
 
      case 'o':  // 输出舵机状态信息
        reportstatus();
        break;
 
      case 'i':  //移动至初始位置
        armInipos();  
        break;

      case 'O' : //机械臂爪子张开
        servocmd('c',0,dsd);
        break;
      
      case 'S' : //机械臂爪子关闭
        servocmd('c',90,dsd);
        break;
      
      case 'H' : //提升机械臂整体运行速度
        dsd -= 5;
        if (dsd < 0) dsd = 0;
        Serial.print("dsd = "); Serial.println(dsd);
        break;
      
      case 'L' : //降低机械臂整体运行速度
        dsd += 5;
        if (dsd < 0) dsd = 0;
        Serial.print("dsd = "); Serial.println(dsd);
        break;
 
      case 'A' : //夹取物体A
        armInipos();//移动到初始位置
        //null;//移动至A位置
        servocmd('c',90,dsd);//机械臂爪子关闭
        //null;//移动
        servocmd('c',0,dsd);//机械臂爪子张开
        break;
      
      case 'B' : //夹取物体B
        armInipos();//移动到初始位置
        //null;//移动至B位置
        servocmd('c',90,dsd);//机械臂爪子关闭
        //null;//移动
        servocmd('c',0,dsd);//机械臂爪子张开
        break;
      
      case 'C' : //夹取物体C
        armInipos();//移动到初始位置
        //null;//移动至C位置
        servocmd('c',90,dsd);//机械臂爪子关闭
        //null;//移动
        servocmd('c',0,dsd);//机械臂爪子张开
        break;
      
      case 'R' : //开始录制
        actionCnt = 0;
        isRecording = true;
        isPlaying = false;
        lastsample = millis();
        Serial.println("Start recording");
        break;

      case 'E' : //结束录制
        isRecording = false;
        Serial.println("Stop recording");
        break;

      case 'P' : //回放动作
        if (actionCnt == 0) Serial.println("You didn't record");
        else {
          isPlaying = true;
          Serial.println("Start replaying");
          replayaction();
        }
        break;

      case 'D' : //清空录制
        actionCnt = 0;
        isRecording = false;
        isPlaying = false;
        Serial.println("Have cleared record");
        break;
      
      default:  //未知指令反馈
        Serial.println("Unknown Command.");
        break;
    }
  }  
}

//控制机械臂运动的函数
void servocmd(char servoname, int topos, int servodelay){  
  
  Servo servogo;  //创建servo对象便于控制
 
  //串口监视器输出接收指令信息
  //Serial.println("");
  //Serial.print("+Command: Servo ");
  //Serial.print(servoname);
  //Serial.print(" to ");
  //Serial.print(topos);
  //Serial.print(" at servoDelay value ");
  //Serial.print(servodelay);
  //Serial.println(".");
  //Serial.println("");
  
  int frompos; //电机起始运动角度值
  
  switch(servoname){
    case 'b':
      if (topos >= basemin && topos <= basemax) {
        servogo = base;
        frompos = base.read();  // 获取当前电机角度值作为电机运动起始角度值
        break;
      } 
      else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
        return;
      }
 
    case 'c':
      if (topos >= clawmin && topos <= clawmax) {    
        servogo = claw;
        frompos = claw.read();  
        break;
      } 
      else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
        return;        
      }
 
    case 'l':
      if (topos >= larmmin && topos <= larmmax) {
        servogo = larm;
        frompos = larm.read();  
        break;
      } 
      else {
        Serial.println("+Warning: lArm Servo Value Out Of Limit!");
        return;
      }
         
    case 'r':
      if(topos >= rarmmin && topos <= rarmmax){
        servogo = rarm;
        frompos = rarm.read();  
        break;
      } 
      else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
        return;
      }      
  }
 
  //指挥电机运行
  if (frompos <= topos){  //如果“起始角度值”小于“目标角度值”
    for (int i=frompos; i<=topos; i++){
      servogo.write(i);
      delay (servodelay);
    }
  }  
  else { //否则“起始角度值”大于“目标角度值”
    for (int i=frompos; i>=topos; i--){
      servogo.write(i);
      delay (servodelay);
    }
  }
}

//移动至初始位置
void armInipos() {
  Serial.println("Restore Initial Position.");
  int robotInipos[4][3] = {
    {'b', 90, dsd},
    {'r', 90, dsd},
    {'l', 20, dsd},
    {'c', 81, dsd} 
  };
 
  for (int i = 0; i < 4; i++){
    servocmd(robotInipos[i][0], robotInipos[i][1], robotInipos[i][2]);
  }
}

//舵机状态信息
void reportstatus() {  
  Serial.println("");
  Serial.println("");
  Serial.println("+ Robot Arm Status Report +");
  Serial.print("Claw Position: "); Serial.println(claw.read());
  Serial.print("Base Position: "); Serial.println(base.read());
  Serial.print("Right  Arm Position:"); Serial.println(rarm.read());
  Serial.print("Left Arm Position:"); Serial.println(larm.read());
  Serial.println("++++++++++++++++++++++++++");
  Serial.println("");
}

//动作录制
void recordaction() {
  if (isRecording) {
    if (actionCnt >= maxActPoints) { // 检查是否已满
      isRecording = false;
      Serial.println("+Warning: Buffer Full, Recording Stopped.");
      Serial.print("Recorded Points: "); Serial.println(actionCnt);
      return;
    }
    unsigned long now = millis();
    if (now - lastsample >= samplebreak) {
      lastsample = now;
      actionBuf[actionCnt].angbase = base.read();
      actionBuf[actionCnt].angrarm = rarm.read();
      actionBuf[actionCnt].anglarm = larm.read();
      actionBuf[actionCnt].angclaw = claw.read();
      actionCnt++;
    }
  }
}


//动作回放
void replayaction() {
  for (int i = 0; i < actionCnt; i++) {
    servocmd('b',actionBuf[i].angbase,dsd);
    servocmd('r',actionBuf[i].angrarm,dsd);
    servocmd('l',actionBuf[i].anglarm,dsd);
    servocmd('c',actionBuf[i].angclaw,dsd);
    delay(samplebreak); 
  }
  isPlaying = false;
  Serial.println("Replay Finished.");
}

//解析坐标控制机械臂移动
bool movetoxyz(float x, float y, float z) {
  float r = sqrt(x * x + y * y);
  float d = sqrt(r * r + z * z);

  // 可达性校验
  if (d > l1 + l2 || d < fabs(l1 - l2)) {
    return false;
  }

  // 底座旋转角 θ₀
  float theta0 = atan2(y, x);

  // 小臂角 θ₂
  float cosalpha = (l1*l1 + l2*l2 - d*d) / (2.0 * l1 * l2);
  cosalpha = constrain(cosalpha, -1.0, 1.0);
  float alpha = acos(cosalpha);
  float theta2 = PI - alpha;

  // 大臂角 θ₁
  float beta = atan2(z, r);
  float cosgamma = (l1*l1 + d*d - l2*l2) / (2.0 * l1 * d);
  cosgamma = constrain(cosgamma, -1.0, 1.0);
  float gamma = acos(cosgamma);
  float theta1 = beta - gamma;

  // 弧度转角度 + 零点偏移 → 实际舵机角度
  int angbase  = round(theta0 * 180.0 / PI) + baseoffset;
  int angrarm = round(theta1 * 180.0 / PI) + rarmoffset;
  int anglarm = round(theta2 * 180.0 / PI) + larmoffset;

  // 硬件限位保护
  angbase  = constrain(angbase, basemin, basemax);
  angrarm = constrain(angrarm, rarmmin, rarmmax);
  anglarm = constrain(anglarm, larmmin, larmmax);

  // 调用原有移动函数，驱动三个舵机
  servocmd('b', angbase, dsd);
  servocmd('r', angrarm, dsd);
  servocmd('l', anglarm, dsd);

  return true;
}