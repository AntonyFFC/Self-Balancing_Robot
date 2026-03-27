// The robot only works with lower clock cpi frequemcy. This code was tested at 80Mhz!
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
TwoWire I2Cone = TwoWire(0);
MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &I2Cone);
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
double setpoint= 178;
double Kp = 10;
double Kd = 0;
double Ki = 0;
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false; 

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define MPU_INT 4

#define L298N_IN1_GPIO 25
#define L298N_IN2_GPIO 26
#define L298N_IN3_GPIO 27
#define L298N_IN4_GPIO 14

void dmpDataReady()
{
    mpuInterrupt = true;
}
void setup() {
  Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    I2Cone.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO); 
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 
    if (devStatus == 0)
    {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        pinMode(MPU_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode (L298N_IN1_GPIO, OUTPUT);
    pinMode (L298N_IN2_GPIO, OUTPUT);
    pinMode (L298N_IN3_GPIO, OUTPUT);
    pinMode (L298N_IN4_GPIO, OUTPUT);
    analogWrite(L298N_IN1_GPIO,LOW);
    analogWrite(L298N_IN2_GPIO,LOW);
    analogWrite(L298N_IN3_GPIO,LOW);
    analogWrite(L298N_IN4_GPIO,LOW);
}
void loop() {
    if (!dmpReady) return;

    static unsigned long startTime = millis();
    if (millis() - startTime < 5000) {
        Stop();
        return;
    }

    if (mpuInterrupt) {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        }
        else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
      
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
            double angle = ypr[1] * 180 / M_PI;
            if(angle < 0) {
                angle += 360;
            }
            input = angle;
        }
    }

    pid.Compute();

    Serial.print(input);
    Serial.print(" =>");
    Serial.println(output);
  
    if (input > 150 && input < 200) {
      if (output > 0)
        Forward();
      else if (output < 0)
        Reverse();
    } else {
        Stop();
    }
    
}
void Forward()
{
    analogWrite(L298N_IN1_GPIO,output);
    analogWrite(L298N_IN2_GPIO,0);
    analogWrite(L298N_IN3_GPIO,output);
    analogWrite(L298N_IN4_GPIO,0);
    Serial.print("F");
}
void Reverse()
{
    analogWrite(L298N_IN1_GPIO,0);
    analogWrite(L298N_IN2_GPIO,output*-1);
    analogWrite(L298N_IN3_GPIO,0);
    analogWrite(L298N_IN4_GPIO,output*-1); 
    Serial.print("R");
}
void Stop()
{
    analogWrite(L298N_IN1_GPIO,0);
    analogWrite(L298N_IN2_GPIO,0);
    analogWrite(L298N_IN3_GPIO,0);
    analogWrite(L298N_IN4_GPIO,0); 
    Serial.print("S");
}
