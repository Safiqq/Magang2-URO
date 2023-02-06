// /arduino/state/hardware dari arduino ke ros
// /control/command/hardware dari ros ke arduino

// FOR ARDUINO MEGA

#include <ros.h>
#include <command/HardwareState.h>
#include <command/HardwareCommand.h>

float pwm1, pwm2, pwm3;
int trig = 4, echo1 = 2, echo2 = 3, echo3 = 18;
int rpwm1 = 5, lpwm1 = 6, rpwm2 = 9, lpwm2 = 10, rpwm3 = 7, lpwm3 = 8;

volatile float distance1, distance2, distance3;
volatile unsigned long timestart1, timeend1, timestart2, timeend2, timestart3, timeend3;

void callbackSub(const command::HardwareCommand& motor_data) {
  // convert pwm [-1 .. 1] ke bentuk 8 bit [0 .. 255]
  pwm1 = motor_data.motor1 * 255;
  pwm2 = motor_data.motor2 * 255;
  pwm3 = motor_data.motor3 * 255;
}

void runMotorDriver(int RPWM, int LPWM, float PWM) {
  // jalankan motor sesuai dengan PWM [0 .. 255]
  if (PWM > 0) {
    analogWrite(RPWM, (int)ceil(PWM));
    digitalWrite(LPWM, LOW);
  } else if (PWM == 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  } else {
    analogWrite(LPWM, (int)ceil(PWM) * (-1));
    digitalWrite(RPWM, LOW);
  }
}

ros::NodeHandle nh;
command::HardwareState sensor_data;
ros::Publisher pub("/arduino/state/hardware", &sensor_data);
ros::Subscriber<command::HardwareCommand> sub("/control/command/hardware", callbackSub);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  // set interrupt dan pinMode untuk tiap sensor dan motor  
  Serial.begin(57600);
  attachInterrupt(digitalPinToInterrupt(echo1), readSensorInt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo2), readSensorInt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echo3), readSensorInt3, CHANGE);

  pinMode(trig, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  pinMode(echo3, INPUT);

  pinMode(rpwm1, OUTPUT);
  pinMode(lpwm1, OUTPUT);
  pinMode(rpwm2, OUTPUT);
  pinMode(lpwm2, OUTPUT);
  pinMode(rpwm3, OUTPUT);
  pinMode(lpwm3, OUTPUT);
}

void loop() {
  //Satu pin trigger untuk 3 sensor biar pembacaannya paralel
  readSensor(trig);

  // assign data jarak dari tiap sensor
  sensor_data.sensor1 = distance1;
  sensor_data.sensor2 = distance2;
  sensor_data.sensor3 = distance3;

  // publish HardwareState.msg dengan rate 20 Hz (50 ms)
  pub.publish(&sensor_data);
  delay(50);
  nh.spinOnce();

  // jalankan motor dengan pwm yg sdh diolah
  runMotorDriver(rpwm1, lpwm1, pwm1);
  runMotorDriver(rpwm2, lpwm2, pwm2);
  runMotorDriver(rpwm3, lpwm3, pwm3);
}

// keluarkan sinyal ultrasonic
void readSensor(int trig) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

// terima hasil waktu dan hitung jarak dari sensor1
void readSensorInt1() {
  // jika echo1 HIGH (kirim sinyal), maka assign waktu awal ke timestart1
  // jika echo1 LOW (terima sinyal), maka hitung waktu tempuh
  if(digitalRead(echo1)) {
    timestart1 = micros(); 
  } else {
    timeend1 = micros();
    int elapsedTime1 = timeend1 - timestart1;
    distance1 = elapsedTime1 * 0.0343/2;
  }
}

// terima hasil waktu dan hitung jarak dari sensor2
void readSensorInt2() {
  if(digitalRead(echo2)) {
    timestart2 = micros(); 
  } else {
    timeend2 = micros();
    int elapsedTime2 = timeend2 - timestart2;
    distance2 = elapsedTime2 * 0.0343/2;
  }
}

// terima hasil waktu dan hitung jarak dari sensor3
void readSensorInt3() {
  if(digitalRead(echo3)) {
    timestart3 = micros(); 
  } else {
    timeend3 = micros();
    int elapsedTime3 = timeend3 - timestart3;
    distance3 = elapsedTime3 * 0.0343/2;
  }
}