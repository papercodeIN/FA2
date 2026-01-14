#include <DabbleESP32.h>
#include <Wire.h> // keep safe if needed for I2C later

// OLED/display logic commented out for later integration
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h> //別途「Adafruit BusIO」ライブラリ必要
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 32
// #define OLED_RESET -1
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <ESP32Servo.h> // use ESP32Servo for servo control

// 変数宣言
int cnt = 0;  //ボタンを押した回数カウント用

// Pin mapping for ESP32 servos (adjust pins if needed)
const uint8_t Srv0 = 5;  // front left
const uint8_t Srv1 = 16; // back left
const uint8_t Srv2 = 17; // front right
const uint8_t Srv3 = 18; // back right

// Servo objects
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3; // kept for compatibility in calls (unused for ledc)
const double PWM_Hz = 50;   //PWM周波数 (kept for reference)
const uint8_t PWM_level = 14; //PWM 14bit(0～16384) (kept for reference)

int pulseMIN = 410;
int pulseMAX = 2048;

int cont_min = 0;
int cont_max = 180;

int angZero[] = {91,95,85,83}; //Trimming
int ang0[4];
int ang1[4];
int ang_b[4];
char ang_c[4];
float ts=100;  //100msごとに次のステップに移る
float td=10;   //20回で分割

int position_status = 0;

// Forward Step
int f_s[4][6]={
  {0,-30,30,0},
  {-30,30,-30,30},
  {-30,0,0,30},
  {30,-30,30,-30}};

// Back Step
int b_s[4][6]={
  {0,30,-30,0},
  {30,-30,30,-30},
  {30,0,0,-30},
  {-30,30,-30,30}};
  
// Left Turn_Step
int l_s[3][6]={
  {-30,30,30,-30},
  {-30,0,0,-30},
  {0,0,0,0}};
  
// Right Turn Step
int r_s[3][6]={
  {30,-30,-30,30},
  {30,0,0,30},
  {0,0,0,0}};
  
// Home position
int h_p[6]={0,0,0,0};

// Up Step
int u_s[6]={-15,-60,15,60};

// Down_Step
int d_s[6]={60,15,-60,-15};

// Right_Down_Step
int rd_s[6]={70,30,0,0};

// Left_Down_Step
int ld_s[6]={0,0,-70,-30};

int angry_state = 0;

void Initial_Value(){  //initial servo angle
  int cn = 50;
  for (int j=0; j <=3; j++){
    Srv_drive(j, angZero[j]);
    ang0[j] = angZero[j];
    ang1[j] = angZero[j];
    delay(cn);
  }
}

// Commented / no-op display functions (so face_* calls compile but do nothing)
void face_clear(){
  // display.clearDisplay();     //表示クリア  <-- commented out
}

void face_center_eye(void *pvParameters){
  // display task removed; not used on plain ESP32 target
  vTaskDelete(NULL); // ensure task ends if accidentally started
}

void face_center(){
  // stub - OLED logic commented
}

void face_up(){
  // stub - OLED logic commented
}

void face_down(){
  // stub - OLED logic commented
}

void face_right(){
  // stub - OLED logic commented
}

void face_left(){
  // stub - OLED logic commented
}

// Replace PWM-ledc servo drive with Servo.write using ESP32Servo objects
void Srv_drive(int srv_CH,int SrvAng){
  int ang = constrain(SrvAng, cont_min, cont_max);
  switch (srv_CH) {
    case 0: servo0.write(ang); break;
    case 1: servo1.write(ang); break;
    case 2: servo2.write(ang); break;
    case 3: servo3.write(ang); break;
    default: break;
  }
}


void servo_set(){
  int a[4],b[4];
  
  for (int j=0; j <=3 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);

      delay(ts/td);
  }
}

void forward_step()
{
  face_center();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + f_s[i][j];
    }
  servo_set();
  }
}

void back_step()
{
  face_center();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + b_s[i][j];
    }
  servo_set();
  }
}

void right_step()
{
  face_right();
  for (int i=0; i <=2 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
  servo_set();
  }
  face_center();
}

void left_step()
{
  face_left();
  for (int i=0; i <=2 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
  servo_set();
  }
  face_center();
}

void up_step()
{
  if(position_status == 0)
  {
    face_up();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + u_s[j];
    }
    servo_set();
    position_status = 1;
  }
  else if(position_status == 1)
  {
    face_center();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + h_p[j];
    }
    servo_set();
    position_status = 0;
  }
}

void down_step()
{
  if(position_status == 0)
  {
    face_down();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + d_s[j];
    }
    servo_set();
    position_status = 1;
  }
  else if(position_status == 1)
  {
    face_center();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + h_p[j];
    }
    servo_set();
    position_status = 0;
  }
}

void right_down_step()
{
  if(position_status == 0)
  {
    face_right();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + rd_s[j];
    }
    servo_set();
    position_status = 1;
  }
  else if(position_status == 1)
  {
    face_center();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + h_p[j];
    }
    servo_set();
    position_status = 0;
  }
}

void left_down_step()
{
  if(position_status == 0)
  {
    face_left();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + ld_s[j];
    }
    servo_set();
    position_status = 1;
  }
  else if(position_status == 1)
  {
    face_center();
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + h_p[j];
    }
    servo_set();
    position_status = 0;
  }
}

void home_position()
{
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + h_p[j];
  }
  servo_set();
}

void setup() {
  Serial.begin(151200);
  // M5.begin(false, false, true); // removed - not used for plain ESP32
  // Wire.begin(21, 22);   //Grove端子をI2C設定(SDA,SCL)  <-- commented, unneeded for now

  // OLED初期設定 commented out (integrate later)
  // if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { ... }
  // display.setTextColor(SSD1306_WHITE);

  Dabble.begin("ESP32-Robot");       // BLE name for Dabble app

  // Attach servos to ESP32 pins
  servo0.attach(Srv0);
  servo1.attach(Srv1);
  servo2.attach(Srv2);
  servo3.attach(Srv3);

  face_center();

  Initial_Value();

  // display task creation removed
  // xTaskCreatePinnedToCore(face_center_eye, "face_center_eye", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // M5.update(); // removed
  // M5 button handling removed; re-add if using M5

  Dabble.processInput(); // mandatory for Dabble

  int a = GamePad.getAngle();
  int b = GamePad.getRadius();

  if (GamePad.isTrianglePressed())
  {
    up_step();
  }

  if (GamePad.isCrossPressed())
  {
    down_step();
  }

  if (GamePad.isCirclePressed())
  {
    right_down_step();
  }

  if (GamePad.isSquarePressed())
  {
    left_down_step();
  }

  if ((position_status == 0)&&(b < 5))
  {
    home_position();
  }
  else if(b >=5)
  {
    if ((a >= 45)&&(a < 135))
    {
      forward_step();
    }
    if ((a >= 135)&&(a < 225))
    {
      left_step();  
    }
    if ((a >= 225)&&(a < 315))
    {
      back_step();
    }
    if ((a >= 315)||(a < 45))
    {
      right_step();
    }
  }
  //delay(100);
}