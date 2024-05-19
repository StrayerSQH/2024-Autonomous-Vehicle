#include    <Servo.h>

// servo control pin
#define MOTOR_PIN           9
// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6
// 74HCT595N chip pin
#define SHCP_PIN            2                               // The displacement of the clock
#define EN_PIN              7                               // Can make control
#define DATA_PIN            8                               // Serial data
#define STCP_PIN            4                               // Memory register clock            

//蜂鸣器控制引脚
#define beepPin        11                     //初始划蜂鸣器引脚
#define beepTimeInterval 1000                 //检测一次的时间间隔   
unsigned long beepTimes = 0;                  //记录设备运行时间
int beepCount = 0;                            //定义一个变量
/****************************************set up and loop part*********************************/
void setup() {
  Serial.begin(115200);                         //设置串口波特率为115200
  pinMode(beepPin, OUTPUT);                   //蜂鸣器引脚设置成输出模式
  Serial.println("设备上线！");                 //串口打印对应的值
}
void loop() {
  ControlBeep();                              //控制蜂鸣器工作
}
/****************************************有源beep part****************************************/
/*控制蜂鸣器工作*/
void ControlBeep() {
  if (millis() - beepTimes >= beepTimeInterval) {
    beepTimes = millis();                    //一定时间执行一次
    beepCount++;
    if (beepCount % 2 == 1) {
      Serial.println("蜂鸣器打开！");
      digitalWrite(beepPin, LOW);            // 蜂鸣器工作
    } else {
      beepCount = 0;
      Serial.println("蜂鸣器关闭！");
      digitalWrite(beepPin, HIGH);          // 蜂鸣器停止工作
    }
  }
}


 
