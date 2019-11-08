#include <ros.h>

#include <std_msgs/Int16.h>

/* 
  Note:
  motor side as front
  IN1 High, IN2 LOW  > forward
  IN1 LOW,  IN2 HIGH > backward
  IN3 HIGH, IN4 LOW  > forward
  IN3 LOW,  IN4 HIGH > backward
*/ 

// Left wheel
#define IN1 4
#define IN2 5
#define ENA 10
// Right wheel
#define IN3 6
#define IN4 7
#define ENB 11

#define PIN_LIGHT A3

ros::NodeHandle nh;

// motor pwm
int pwm_r = 0, pwm_l = 0;

// callback
void cb_r(const std_msgs::Int16& msg){
  pwm_r = msg.data;
  //nh.loginfo("pwm r update");
}

void cb_l(const std_msgs::Int16& msg){
  pwm_l = msg.data;
  //nh.loginfo("pwm l update");
} 


ros::Subscriber<std_msgs::Int16> sub_right("/arg3/right_pwm", &cb_r);
ros::Subscriber<std_msgs::Int16> sub_left("/arg3/left_pwm", &cb_l);
std_msgs::Int16 light_msg;
ros::Publisher pub_light_val("/arg3/light_value", &light_msg);

unsigned long t_stamp;

void setup()
{
  // Init ros node handler
  nh.initNode();
  // Subscribers
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.advertise(pub_light_val);
  // Set pin mode
  pinMode(OUTPUT, IN1);
  pinMode(OUTPUT, IN2);
  pinMode(OUTPUT, IN3);
  pinMode(OUTPUT, IN4);
  pinMode(OUTPUT, ENA);
  pinMode(OUTPUT, ENB);

  t_stamp = millis();
}

void loop()
{
  // Forward
  if(pwm_r >= 0 and pwm_l >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, pwm_r);
  }
  // Left
  else if(pwm_r >= 0 and pwm_l <= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, pwm_l);
    analogWrite(ENB, abs(pwm_r));
  }
  // Right
  else if(pwm_r <= 0 and pwm_l >= 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, pwm_r);
  }
  // Backward
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, abs(pwm_l));
    analogWrite(ENB, abs(pwm_r));
  }

  if(t_stamp - millis() > 200){
    light_msg.data = analogRead(PIN_LIGHT);
    pub_light_val.publish(&light_msg);
    t_stamp = millis();
  }
  
  nh.spinOnce();
}
