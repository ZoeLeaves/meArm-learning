#include <Servo.h>                //使用servo库
Servo base, larm, rarm, claw ;    //创建4个servo对象

// 动作点：存储4个舵机的角度
struct ActionPoint { 
  int angbase,angrarm,anglarm,angclaw;
};
struct DrawPoint {
  float x,y,z;
};
//存储电机极限值
const int basemin = 0;
const int basemax = 180;
const int rarmmin = 68;
const int rarmmax = 145;
const int larmmin = 20;
const int larmmax = 115;
const int clawmin = 81;
const int clawmax = 178;
const int maxActPoints = 100; // 最大录制动作点数
const int maxDrawPoints = 5; //最大绘图点数
const float l1 = 8.0; //大臂长
const float l2 = 8.0; //小臂长
const float h = 6.0; //底座高度
const int baseoffset = 0;
const int rarmoffset = 135;
const int larmoffset = 50;
int dsd = 15; //运动延迟时间，用于控制电机运行速度
int step = 1; //控制摇杆移动步长
int samplebreak = 200; // 采样间隔（毫秒），越小动作越流畅
int actionCnt = 0; //已录制动作点数
int drawCnt = 0; //已记录示教点数
unsigned long lastsample = 0; // 上一次采样的时间
int z_up = 10; //抬笔高度
ActionPoint actionBuf[maxActPoints]; // 动作缓冲区
DrawPoint drawBuf[maxDrawPoints]; //示教点存放

int state1 = 2; //循环按钮1计数
int state2 = 1; //循环按钮2计数
bool mode = 1; //1为指令模式，0为绘图模式
bool isRecording = false; // 录制状态标志
bool isPlaying = false; // 回放状态标志
bool isdrawing = false; //绘图状态标志
bool ispaused = false; //暂停状态标志

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
  armInipos();
  //base.write(90); 
  //delay(10);
  //larm.write(20); 
  //delay(10);
  //rarm.write(90); 
  //delay(10);
  //claw.write(81);  
  //delay(10); 
  
  Serial.begin(9600);
  Serial.println("Robot Arm Set Up");
}

void loop() {
  
  if (Serial.available()>0) {  
    char serialcmd = Serial.read();
    if ( mode == 1 ) armdatacmd(serialcmd); //指令模式
    else armdrawcmd(serialcmd); //绘图模式
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
    if(sscanf(buf.c_str(), "%d,y%d,z%d", &x,&y,&z) == 3) {
      servocmd('b', x, dsd);
      servocmd('l', y, dsd);
      servocmd('r', z, dsd);
      //movetoxyz(x,y,z);
    }
    }
  else {
    switch(serialcmd) {   
 
      case 'm' : //切换到绘图模式
        mode = 0;
        break;

      case 'o':  // 输出舵机状态信息
        reportstatus();
        break;
 
      case 'i':  //移动至初始位置,按键4功能
        armInipos();  
        break;

      case 'O' : //机械臂爪子张开
        servocmd('c',81,dsd);
        break;
      
      case 'S' : //机械臂爪子关闭
        servocmd('c',178,dsd);
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
        servocmd('c',81,dsd);//机械臂爪子张开
        servocmd('b',129,dsd);//移动至A位置
        servocmd('l',112,dsd);
        servocmd('r',81,dsd);
        servocmd('c',178,dsd);//机械臂爪子关闭
        armInipos();//移动到初始位置
        servocmd('b',51,dsd);//移动至A'位置
        servocmd('l',112,dsd);
        servocmd('r',81,dsd);
        servocmd('c',81,dsd);//机械臂爪子张开
        armInipos();//移动到初始位置
        break;
      
      case 'B' : //夹取物体B
        armInipos();//移动到初始位置
        servocmd('c',81,dsd);//机械臂爪子张开
        servocmd('b',159,dsd);//移动至B位置
        servocmd('l',106,dsd);
        servocmd('r',85,dsd);
        servocmd('c',178,dsd);//机械臂爪子关闭
        armInipos();//移动到初始位置
        servocmd('b',20,dsd);//移动至B'位置
        servocmd('l',106,dsd);
        servocmd('r',85,dsd);
        servocmd('c',81,dsd);//机械臂爪子张开
        armInipos();//移动到初始位置
        break;
      
      case 'C' : //夹取物体C
        armInipos();//移动到初始位置
        servocmd('c',81,dsd);//机械臂爪子张开
        servocmd('b',112,dsd);//移动至C位置
        servocmd('l',111,dsd);
        servocmd('r',85,dsd);
        servocmd('c',178,dsd);//机械臂爪子关闭
        armInipos();//移动到初始位置
        servocmd('b',68,dsd);//移动至C'位置
        servocmd('l',111,dsd);
        servocmd('r',85,dsd);
        servocmd('c',81,dsd);//机械臂爪子张开
        armInipos();//移动到初始位置
        break;
      
      case 'K' : //按键1功能
        state1 = (state1 + 1) % 3;
        switch (state1) {

          case 0 : //夹取A物体
            armInipos();//移动到初始位置
            servocmd('c',81,dsd);//机械臂爪子张开
            servocmd('b',129,dsd);//移动至A位置
            servocmd('l',112,dsd);
            servocmd('r',81,dsd);
            servocmd('c',178,dsd);//机械臂爪子关闭
            armInipos();//移动到初始位置
            servocmd('b',51,dsd);//移动至A'位置
            servocmd('l',112,dsd);
            servocmd('r',81,dsd);
            servocmd('c',81,dsd);//机械臂爪子张开
            armInipos();//移动到初始位置
            break;
          
          case 1 : //夹取B物体
            armInipos();//移动到初始位置
            servocmd('c',81,dsd);//机械臂爪子张开
            servocmd('b',160,dsd);//移动至B位置
            servocmd('l',106,dsd);
            servocmd('r',85,dsd);
            servocmd('c',178,dsd);//机械臂爪子关闭
            armInipos();//移动到初始位置
            servocmd('b',20,dsd);//移动至B'位置
            servocmd('l',106,dsd);
            servocmd('r',85,dsd);
            servocmd('c',81,dsd);//机械臂爪子张开
            armInipos();//移动到初始位置
            break;

          case 2 : //夹取C物体
            armInipos();//移动到初始位置
            servocmd('c',81,dsd);//机械臂爪子张开
            servocmd('b',112,dsd);//移动至C位置
            servocmd('l',111,dsd);
            servocmd('r',85,dsd);
            servocmd('c',178,dsd);//机械臂爪子关闭
            armInipos();//移动到初始位置
            servocmd('b',68,dsd);//移动至C'位置
            servocmd('l',111,dsd);
            servocmd('r',85,dsd);
            servocmd('c',81,dsd);//机械臂爪子张开
            armInipos();//移动到初始位置
            break;

        }
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

      case 'T' : //按键2功能
        state2 = (state2 + 1) % 2;
        switch (state2) {

          case 0 :
            actionCnt = 0;
            isRecording = true;
            isPlaying = false;
            lastsample = millis();
            Serial.println("Start recording");
            break;
          
          case 1 :
            isRecording = false;
            Serial.println("Stop recording");
            break;

        }
        break;

      case 'P' : //回放动作，按键3功能
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

      case 'Q' : //绘制直线
        movetoxyz(0,10,20);
        servocmd('c',81,dsd);//机械臂爪子张开
        servocmd('c',178,dsd);//机械臂爪子关闭
        movetoxyz(0,10,20);
        drawline(0,10,20,21,10,20);
        armInipos();
        break;

      case 'N' : //
        movetoxyz(10,10,10);
        servocmd('c',81,dsd);//机械臂爪子张开
        servocmd('c',178,dsd);//机械臂爪子关闭
        movetoxyz(10,10,0);
        drawline(10,10,0,10,20,0);
        drawline(10,20,0,20,10,0);
        drawline(20,10,0,20,20,0);
        armInipos();
        break;

      case 'F' : //坐标调试
        movetoxyz(15,10,10);
        delay(10);
        movetoxyz(20,10,10);
        delay(10);
        movetoxyz(25,10,10);
        delay(10);
      
      default:  //未知指令反馈
        Serial.println("Unknown Command.");
        break;
    }
  }  
}

//绘图模式
void armdrawcmd(char serialcmd) {
  switch (serialcmd) {
    
    case 'K' : //记录示教点
      if (!isdrawing && drawCnt < maxDrawPoints) {
        float cx,cy,cz;
        xyzforAng(&cx,&cy,&cz);//通过坐标计算舵机角度
        drawBuf[drawCnt].x = cx;
        drawBuf[drawCnt].y = cy;
        drawBuf[drawCnt].z = cz;
        drawCnt++;
        Serial.print("Have saved point,points:");
        Serial.println(drawCnt);
      }
    
    case 'T' : //按键2功能
        state2 = (state2 + 1) % 2;
        switch (state2) {
          
          case 0: // 暂停
          if(isdrawing == true) ispaused = true;
          break;

          case 1: // 继续
          if(isdrawing == true && ispaused == true) ispaused = false;
          break;

        }
        break;
      
    case 'i': //取消，回到初始位置
      if(isdrawing == true) {
        isdrawing = false;
        ispaused = false;
        armInipos();
      }
      break;

    case 'P' : //启动绘图
      if (!isdrawing && drawCnt == maxDrawPoints) {
        Serial.println("Start drawing");
        isdrawing = true;
        ispaused = false;
        armInipos();
        movetoxyz(drawBuf[0].x, drawBuf[0].y, z_up);
        movetoxyz(drawBuf[0].x, drawBuf[0].y, drawBuf[0].z); //落笔
        for (int i=0;i<maxDrawPoints-1;i++) {
          bool finish = drawline(drawBuf[i].x,drawBuf[i].y,drawBuf[i].z,drawBuf[i+1].x,drawBuf[i+1].y,drawBuf[i+1].z);
          if (!finish) break; //中途取消，退出绘图循环
        }
          movetoxyz(drawBuf[4].x, drawBuf[4].y, z_up); //绘图结束抬笔
          isdrawing = false;
          ispaused = false;
          Serial.println("Finish drawing");
          armInipos();
      }
      break;
  }
}

//画直线的函数
bool drawline(float x0,float y0,float z0,float x1,float y1,float z1)
{
  float dx = x1-x0;
  float dy = y1-y0;
  float dz = z1-z0;
  float total = sqrt(dx*dx + dy*dy + dz*dz);

  int steps = ceil(total / step); //所需要步数
  for(int i=0; i<=steps; i++) {
    if(!isdrawing) return false; // 取消任务，直接退出
    while(ispaused) { // 如果暂停，原地等待，直到解除暂停或者取消
      if(!isdrawing) return false;
      delay(50);
    }
    float ratio = (float)i / steps; //得出每一步移动的比例
    float x = x0 + ratio*dx;
    float y = y0 + ratio*dy;
    float z = z0 + ratio*dz;
    movetoxyz(x,y,z);
    delay(40);
  }
  return true;
}

//控制机械臂运动的函数
void servocmd(char servoname, int topos, int servodelay) {  
  
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
    {'c', 178, dsd} 
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


bool movetoxyz(float x, float y, float z) {
  float tmp1;
  float tmp2;
  float H1 = -1*z + h;
  float r = sqrtf(x*x+ y*y) - 6.3;
  float l3 = sqrtf(H1*H1 + r*r);

  if(l3>l1+l2) return false;
  if(l3<fabs(l1- l2)) return false;
  if(r<0) return false;
  int angbase = atan2f(y, x) * 180.0 / PI;
  float tmp3 = (l1*l1 + l2*l2 - l3*l3) / (2*l1*l2);
  tmp3 = constrain(tmp3, -1.0, 1.0);
  float tmp4 = (l3*l3 + l1*l1 - l2*l2)/ (2*l3*l1);
  tmp4 = constrain(tmp4, -1.0, 1.0);
  tmp1 = acosf(tmp4) * 180.0 / PI;
  tmp2 = atan2f(H1,r) * 180.0 /PI;
  int angrarm = tmp1 + tmp2;
  int anglarm = (acosf(tmp3) * 180.0 / PI) + angrarm;
  base.write(angbase+4.6);
  rarm.write(-1*angrarm+135);
  larm.write(anglarm+50);
  return true;
}


//解析坐标控制机械臂移动
/*bool movetoxyz(float x, float y, float z0) {
  // ---- 1. 换算到"大臂根部"坐标系 ----
  float z = z0 - h;
  float r = sqrt(x * x + y * y);
  float d = sqrt(r * r + z * z);

  // ---- 2. 可达性校验 + 除零保护（打印必须在 return 之前！）----
  if (d > l1 + l2 || d < fabs(l1 - l2) || d < 1e-6) {
    Serial.print("Can't move to. d="); Serial.println(d);
    return false;
  }

  // ---- 3. 平面三角形三个角（和 exchangeData 完全同一套）----
  float beta  = atan2(z, r);                                        // 目标极角
  float gamma = acos(constrain((l1*l1 + d*d - l2*l2) / (2.0*l1*d), -1.0, 1.0));
  float alpha = acos(constrain((l1*l1 + l2*l2 - d*d) / (2.0*l1*l2), -1.0, 1.0));

  // ---- 4. 三个输出角（肘上分支）----
  float theta0 = atan2(y, x);    // 底座回转角
  float theta1 = beta + gamma;   // 大臂绝对角（= 你 exchangeData 的 rarmangle）
  float farm   = alpha + theta1; // 小臂绝对角 + 180°（= 你 exchangeData 的 farmangle）

  // ---- 5. 转舵机读数（沿用你源代码的骨架）----
  int angbase =  round(theta0 * 180.0 / PI) + baseoffset;
  int angrarm = -round(theta1 * 180.0 / PI) + rarmoffset;
  int anglarm =  round(farm   * 180.0 / PI) + larmoffset;   // ★原来是 theta2 - larmoffset，改这里

  // ---- 6. 限位：先判断，超了就拒绝，不要 constrain 静默压边界 ----
  if (angbase < basemin || angbase > basemax ||
      angrarm < rarmmin || angrarm > rarmmax ||
      anglarm < larmmin || anglarm > larmmax) {
    Serial.print("Joint limit! b/r/l = ");
    Serial.print(angbase); Serial.print('/');
    Serial.print(angrarm); Serial.print('/');
    Serial.println(anglarm);
    return false;
  }

  // ---- 7. 下发 ----
  base.write(angbase);
  rarm.write(angrarm);
  larm.write(anglarm);
  return true;
}*/

//利用舵机角度求坐标
bool xyzforAng(float *cx, float *cy, float *cz) {
  // 读取当前舵机目标角
  int angBase = base.read();
  int angRarm = rarm.read();
  int angLarm = larm.read();

  // 舵机读数 → 数学角（必须是逆解映射的严格反演）
  float th0  =  (angBase - baseoffset) * PI / 180.0;   // 底座角
  float th1  = -(angRarm - rarmoffset) * PI / 180.0;   // ★大臂绝对角：补负号
  float farm =  (angLarm - larmoffset) * PI / 180.0;   // 小臂绝对角 + 180°
  float phi  = farm - PI;                              // ★小臂绝对角 = farm − 180°，不加 th1

  // 正运动学（绝对角结构：两角都从水平量起，互相独立）
  float r = l1 * cos(th1) + l2 * cos(phi);
  float z = l1 * sin(th1) + l2 * sin(phi) + h;

  *cx = r * cos(th0);
  *cy = r * sin(th0);
  *cz = z;
  return true;
}