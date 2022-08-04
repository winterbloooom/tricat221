#define ip2 2 // 서보모터 채널1 
#define ip1 3 // 스러스터 체널2
#define ip5 4 // 변환 채널5
#define ip6 5 // 릴레이채널6

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
 
int ch1; //스러스터  
int ch2; //서보모터환

int ch5; //LED
int ch6; //릴레이모듈
int val1;
int val2;
int relaypin = 10;



Servo thruster1; //스러스터1
Servo thruster2; //스러스터2
Servo servo1;
Servo servo2;    //서보모터

void Servo_cb( const std_msgs::UInt16& cmd_msg){
  servo1.write(cmd_msg.data);
  servo2.write(cmd_msg.data-1);
}                             //0-300

void thruster_cb( const std_msgs::UInt16& cmd_msg){
  thruster1.writeMicroseconds(cmd_msg.data);
  thruster2.writeMicroseconds(cmd_msg.data);//1100-1900
}

ros::Subscriber<std_msgs::UInt16> sub1("servo", Servo_cb);
ros::Subscriber<std_msgs::UInt16> sub3("thruster", thruster_cb);

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub3);
  
  pinMode(relaypin,OUTPUT); // 릴레이를 출력으로 설정
  pinMode(ip6,INPUT_PULLUP); // 스위치(릴레이)를 입력으로 설정
  pinMode (ip1, INPUT);

  thruster1.attach(6); // 스러스터 6번핀
  thruster2.attach(7);// 스러스터2 7번핀
  thruster1.writeMicroseconds(1500); // send "stop" signal to ESC
  thruster2.writeMicroseconds(1500); 
  servo1.attach(8); //서보모터 8번핀
  servo2.attach(9); //서보모터 9번핀
  servo1.writeMicroseconds(95); // initial state is Neutral
  servo2.writeMicroseconds(95); 
  
  //delay(1000);
  //Serial.begin(9600);
}

void loop()
{
    ch5 = pulseIn(ip5,HIGH);
  {
    if(ch5 < 1450)
      {
        autonomous();
      }
    else if(ch5 > 1450)
      {
        rc();
      }
  }

}
