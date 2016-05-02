#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle  nh;

int xpos = 250;
int ypos = 175;
int xserv = 327;
int yserv = 327;
int centxmax = 275;
int centymax = 200;
int centxmin = 225;
int centymin = 150;

void servox( const std_msgs::UInt16& cmd_msg)
{
  xpos = cmd_msg.data;  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
  if(xpos < centxmin)
    {
      if(xserv < 525)
      {
       xserv = xserv + 1;  
      }
    }
    else if(xpos > centxmax)
    {
      if(xserv > 125)
      {
       xserv = xserv - 1; 
      }
    }
    pwm.setPWM(6,0,xserv);
    Serial.println("x = ");
    Serial.println(xserv);   
}
void servoy( const std_msgs::UInt16& cmd_msg)
{
  ypos = cmd_msg.data;  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
  if(ypos < centymin)
    {
      if( yserv > 237)
      {
        yserv = yserv - 1;
      }
    }
    else if(ypos > centymax)
    {
      if( yserv < 341)
      {
        yserv = yserv + 1;
      }
    }
    pwm.setPWM(8,0,yserv); 
    Serial.println("y = ");
    Serial.println(yserv); 
}


ros::Subscriber<std_msgs::UInt16> sub1("/face_detector/face_position_X", servox);
ros::Subscriber<std_msgs::UInt16> sub2("/face_detector/face_position_Y", servoy);

void setup()
{
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  #ifdef ESP8266
    Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
  #endif
  pwm.begin(); 
  pwm.setPWMFreq(60);

}

void loop(){
  nh.spinOnce();
  delay(1);
}
