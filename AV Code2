#include <Servo.h>

const char* WIFI_SSID = "Redmi K50";
const char* WIFI_PASS = "sqh5201314";

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
// 超声波控制引脚
#define Trig_PIN            12
#define Echo_PIN            13
//蜂鸣器控制引脚
#define Beep_PIN            10
// 循迹控制引脚
#define LEFT_LINE_TRACJING      A0
#define CENTER_LINE_TRACJING    A1
#define right_LINE_TRACJING     A2

Servo MOTORservo;
//WiFiServer server(88)
//WiFiClient client;

////////////////////////////////////不能进行修改/////////////////////////////////////////////////
const int Forward       = 92;                               // forward 前进
const int Backward      = 163;                              // back 后退
const int Turn_Left     = 149;                              // left translation 左平移
const int Turn_Right    = 106;                              // Right translation 右平移
const int Top_Left      = 20;                               // Upper left mobile 左上移
const int Bottom_Left   = 129;                              // Lower left mobile 左下移
const int Top_Right     = 72;                               // Upper right mobile 右上
const int Bottom_Right  = 34;                               // The lower right move 右下
const int Contrarotate  = 172;                              // Counterclockwise rotation 逆时针转
const int Clockwise     = 83;                               // Rotate clockwise 顺时针转
const int Stop          = 0;                                // stop 停止
const int Moedl1        = 25;                               // model1
const int Moedl2        = 26;                               // model2
const int Moedl3        = 27;                               // model3
const int Moedl4        = 28;                               // model4
const int MotorLeft     = 230;                              // servo turn left舵机左转
const int MotorRight    = 231;                              // servo turn rig ht 舵机右转
////////////////////////////////////不能进行修改/////////////////////////////////////////////////

unsigned long previousMillis_Tracking_Stop = 0;
unsigned long previousMillis_Tracking_Beep = 0;

char direction;

int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line = 600;

int leftDistance = 10;
int middleDistance = 10;
int rightDistance = 10;

byte RX_package[3] = {0};
uint16_t angle = 0;
byte order = Stop;
int model_var = 3;
int UT_distance = 0;

void setup()
{
    Serial.setTimeout(10);
    Serial.begin(115200);

    MOTORservo.attach(MOTOR_PIN);

    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);

    pinMode(Trig_PIN, OUTPUT);
    pinMode(Echo_PIN, INPUT);

    pinMode(LEFT_LINE_TRACJING, INPUT);
    pinMode(CENTER_LINE_TRACJING, INPUT);
    pinMode(right_LINE_TRACJING, INPUT);

    pinMode(Beep_PIN, OUTPUT);

    MOTORservo.write(angle);

    Motor(Stop, 0);

    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    //   delay(1000);
    //   Serial.println("正在连接到Wi-Fi网络...");
    // }

    // Serial.println("成功连接到Wi-Fi网络");
    // Serial.print("Wi-Fi模块的IP地址：");
    // Serial.println(WiFi.localIP());

    // server.begin();
}

void loop()
{
    digitalWrite(Beep_PIN, HIGH); // 停止鸣叫
    switch (model_var)
    {
      case 0://启动
        model_var = Start();
        break;
      case 1:// Tracking model 巡线
        model_var = Tracking();      
        break;
      case 2://逆时针旋转伺服电机
        model_var = ContrarotateServo();
        break;
      case 3:// Counterclockwise spin motion 逆时针旋转
        model_var = Counterclockwise_spin_motion();
        break;
      case 4://蜂鸣器模块
        model_var = BeepModel(10);
        break;
      case 5://摄像头识别颜色范围模块
        model_var = ColorDetect();
        break;
      case 6://向左横走
        model_var = HengZuo();
        break;
    }
}

int Start()
{
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  while (elapsedTime < 1000) 
  {
    Motor(Forward, 250);
    elapsedTime = millis()-startTime;
  }

  Tracking();

  //return 1;
}

int Tracking()      // 循迹模组
{
  digitalWrite(Beep_PIN, HIGH); // 停止鸣叫
  int judge = 1;
  while(judge){
    //MOTORservo.write(90);
    Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
    Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
    Right_Tra_Value = analogRead(right_LINE_TRACJING);
    if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Forward, 250);//175
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 230);//165
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 250);//190
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 250);//190
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 230);//165
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Stop, 0);
        judge = 0;
    }
  }
  
  int BeepTime = 10;
  BeepModel(BeepTime); 
}

int ContrarotateServo()//伺服电机逆时针旋转90°
{
    MOTORservo.write(angle);
    angle += 1;
    if (angle >= 180)
    {
        angle = 180;
    }
    delay(10);
}

int Counterclockwise_spin_motion()      // 逆时针旋转
{
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  while (elapsedTime < 525) 
  {
    Motor(Contrarotate, 250);
    elapsedTime = millis()-startTime;
    Serial.println(elapsedTime);
  }
  Motor(Stop, 0);

  //ColorDetect();
  //HengZuo();
}

int Clockwise_spin_motion()
{
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  while (elapsedTime < 525) 
  {
    Motor(Clockwise, 250);
    elapsedTime = millis()-startTime;
    Serial.println(elapsedTime);
  }
  Motor(Stop, 0);
  
}

int HengZuo()
{
  int counter = 0;
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  while (elapsedTime < 2300 * 6) {
    if (elapsedTime % 4600 < 2300) {
      Motor(Turn_Left, 500);
    } else {
      Motor(Stop, 0);
      //ColorDetect();
      delay(2300);
    }
    elapsedTime = millis() - startTime;
  }

  Counterclockwise_spin_motion();
  Counterclockwise_spin_motion();
  HengZuo();
  
  //return 6;
}

int BeepModel(int BeepTime)
{
  unsigned long startTime = millis();
  unsigned long elapsedTime = 200;
  
  digitalWrite(Beep_PIN, HIGH); // 停止鸣叫
  
  while (elapsedTime < 200 * BeepTime * 2) {
    if (elapsedTime % 400 < 200) {
      digitalWrite(Beep_PIN, LOW);
    } else {
      digitalWrite(Beep_PIN, HIGH);
    }
    elapsedTime = millis() - startTime;
  }
  
  digitalWrite(Beep_PIN, HIGH); // 停止鸣叫

  delay(2000);
  
  HengZuo();
  //Counterclockwise_spin_motion();
}

int ColorDetect()
{
  int judge = 1;
  while(judge)
  {
    if (Serial.available())
    {
      Serial.println(direction);
      direction = Serial.read();
      Serial.println(direction);
      if (direction == 'l')
      {
        Counterclockwise_spin_motion();
        break;
      }
      else if (direction == 'r')
      {
        Clockwise_spin_motion();
        break;
      }
    }
  }
}

void Motor(int Dir, int Speed)      // motor drive
{
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}

float SR04(int Trig, int Echo)      // ultrasonic measured distance
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float distance = pulseIn(Echo, HIGH) / 58.00;
    delay(10);
    
    return distance;
}


void RXpack_func()  //Receive data
{
    if(Serial.available() > 0)
    {
        delay(1);                                           // 延时 1MS
        if(Serial.readBytes(RX_package, 3))
        {
            if (RX_package[0] == 0xA5 && RX_package[2] == 0x5A)     // 验证了包头与包尾
            {
                order = RX_package[1];
                if(order == Moedl1) 
                {
                    model_var = 0;
                }
                else if (order == Moedl2)
                {
                    model_var = 1;
                }
                else if (order == Moedl3)
                {
                    model_var = 2;
                }
                else if (order == Moedl4)
                {
                    model_var = 3;
                }
                //////////////////////////////
                // switch (RX_package[1])
                // {
                // case Stop:
                //     Serial.println("Stop");
                //     break;
                // case Forward:
                //     Serial.println("Forward");
                //     break;
                // case Backward:
                //     Serial.println("Backward");
                //     break;
                // case Turn_Left:
                //     Serial.println("Turn_Left");
                //     break;
                // case Turn_Right:
                //     Serial.println("Turn_Right");
                //     break;
                // case Top_Left:
                //     Serial.println("Top_Left");
                //     break;
                // case Bottom_Left:
                //     Serial.println("Bottom_Left");
                //     break;
                // case Top_Right:
                //     Serial.println("Top_Right");
                //     break;
                // case Bottom_Right:
                //     Serial.println("Bottom_Right");
                //     break;
                // case Clockwise:
                //     Serial.println("Clockwise");
                //     break;
                // case MotorLeft:
                //     Serial.println("MotorLeft");
                //     break;
                // case MotorRight:
                //     Serial.println("MotorRight");
                //     break;
                // case Moedl1:
                //     Serial.println("Moedl1");
                //     break;
                // case Moedl2:
                //     Serial.println("Moedl2");
                //     break;
                // case Moedl3:
                //     Serial.println("Moedl3");
                //     break;
                // case Moedl4:
                //     Serial.println("Moedl4");
                //     break;
                // default:
                //     break;
                // }
            }
        }
    }
}



