//표준 PID 제어 함수 구현(이중루프 PID 제어기 X)
//목표 : roll, pitch, yaw의 각도에 대한 pid 출력값 구하기
#include <Thread.h>
#include <Wire.h>
#include <SoftwareSerial.h> // L293D 모터 드라이브 라이브러리
#include <AFMotor.h>          // 서보모터 라이브러리

#define sensitivity 5
#define BT_RXD 13
#define BT_TXD 12
SoftwareSerial btSerial(BT_RXD, BT_TXD);

Thread SendingThread = Thread();

AF_DCMotor motor_1(1);       
AF_DCMotor motor_2(2);  
AF_DCMotor motor_3(3);       
AF_DCMotor motor_4(4);  

float c = 0;
float SendingPitch;
int Speed = 30;
char value = 0;

int trig = 8;
int echo = 9;




const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  float gyro_x, gyro_y, gyro_z ;


  float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
//float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
extern float roll_output, pitch_output, yaw_output;

void setup() {
  Serial.println("test!!");
  initMPU6050(); //가속도 자이로 센서 값을 읽음
  Serial.begin(9600);
  btSerial.begin(9600);
  calibAccelGyro(); //센서 보정 루틴
  initDT();// 시간 간격에 대한 초기화
  
  motor_1.run(RELEASE);    
  motor_2.run(RELEASE);         
  motor_3.run(RELEASE);    
  motor_4.run(RELEASE);  

  SendingThread.onRun(SendToAPPCallback);
  SendingThread.setInterval(500);
}
   
//MPU-6050초기화 

void SendToAPPCallback(){
  btSerial.write(filtered_angle_x);
  Serial.print("filtered_angle_x : ");
  Serial.print(filtered_angle_x);
  Serial.print(" Acc : ");
  Serial.print(AcX);
  Serial.print(", ");
  Serial.print(AcY);
  Serial.print(", ");
  Serial.println(AcZ);
}

void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
  
 //가속도 자이로 센서를 읽음
void readAccelGyro()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}
  
//센서 들의 기본값들의 평균을 내야하는 루틴(센서 보정 루틴)
void calibAccelGyro() 
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  
  //가속도 자이로 센서를 읽어들임
  readAccelGyro();
 
  //읽어드렸으면 이제 읽어드린 값을 토대로 평균값을 구하면 됨
  for(int i=0; i<10; i++)
  {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);//0.1초
  }
  //맨 처음 기본 센서 값들을 보여지고 그다음에 평균값을 구하는 함수
  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;
  
  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}
  
 //프로세싱 스케치로 각도 정보를 보내는 루틴

unsigned long t_now;
unsigned long t_prev;
 
void initDT(){
  t_prev = millis();
}
 
void calcDT(){
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0;
  t_prev = t_now;
}
 
void calcAccelYPR() //가속도 센서 처리 루틴
{
  float accel_x, accel_y, accel_z; //x, y, z 축에 대한 각도 저장 변수
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;
 
  accel_x = AcX - baseAcX;
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);
 
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_x = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;
 
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_y = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;
 
  accel_angle_z = 0;
}

void calcGyroYPR()
{
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131; // 각속도를 저장하는 변수

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

}

void calcFilteredYPR() //상보필터
{
   const float ALPHA = 0.96;
   float tmp_angle_x, tmp_angle_y, tmp_angle_z; //임시 각도

   tmp_angle_x = filtered_angle_x + gyro_x * dt; // 이전 보정 각도 + 현재 자이로 센서를 이용해 얻은 각도(gyro_x * dt)
   tmp_angle_y = filtered_angle_y + gyro_y * dt;
   tmp_angle_z = filtered_angle_z+ gyro_z * dt;

   filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
   filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
   filtered_angle_z = tmp_angle_z;
}

//################## loop #####################
void loop() {
  if (btSerial.available()) { //8번핀에 HC06으로부터 데이터가 들어오면
    value = btSerial.read(); // 데이터를 읽어서 data 변수에 저장
    Serial.write(value); 
    Serial.write("\n");
  }

  if(value == '1'){
    c = 20;
//    c = 16000/180 * c;
    Serial.print(30);
    Serial.println("degrees");
    value = '0';
    
Serial.println(c);
  }
  else if(value == '2'){
    c = 40;
//    c = 16000/180 * c;
    Serial.print(60);
    Serial.println("degrees");
    value = '0';
  }
  else if(value == '3'){
    c = 60;
//    c = 16000/180 * c;
    Serial.print(90);
    Serial.println("degrees");
    value = '0';
  }

  if(value == '4'){
    c = -20;
//    c = 16000/180 * c;
    Serial.print(30);
    Serial.println("degrees");
    value = '0';
    
Serial.println(c);
  }
  else if(value == '5'){
    c = -40;
//    c = 16000/180 * c;
    Serial.print(60);
    Serial.println("degrees");
    value = '0';
  }
  else if(value == '6'){
    c = -60;
//    c = 16000/180 * c;
    Serial.print(90);
    Serial.println("degrees");
    value = '0';
  }
    else if(value == '7'){
    c = 0;
    c = 16000/180 * c;
    Serial.print(90);
    Serial.println("degrees");
    value = '0';
  }
  
  else if(value == 'a'){
    Speed = 50;
    value = '0';
  }
  
  else if(value == 'b'){
    Speed = 100;
    value = '0';
  }

    else if(value == 'c'){
    Speed = 150;
    value = '0';
  }

    else if(value == 'd'){
    Speed = 200;
    value = '0';
  }

  readAccelGyro();
  calcDT(); //시간 간격 계산
  calcAccelYPR(); //가속도 센서 처리 루틴
  calcGyroYPR(); //자이로 센서 처리 루틴 --> 가속도 센서의 값을 해석하기 위해 Roll, Pitch, Yaw에 대한 각도 구하는 함수
  calcFilteredYPR(); //상보필터 적용


if(SendingThread.shouldRun())
  SendingThread.run();

if( filtered_angle_x >= -sensitivity + c && filtered_angle_x <= sensitivity + c )
 {
motor_1.run(RELEASE); 
  }
  
else if( filtered_angle_x > sensitivity + c)
 {
 motor_1.run(FORWARD);    
 motor_1.setSpeed(Speed);   
 }
  
if( filtered_angle_x >= -sensitivity + c && filtered_angle_x <= sensitivity + c )
 {
motor_3.run(RELEASE); 
  }
  
else if(filtered_angle_x <= -sensitivity + c )
 {
 motor_3.run(FORWARD);    
 motor_3.setSpeed(Speed);   
 }
}
