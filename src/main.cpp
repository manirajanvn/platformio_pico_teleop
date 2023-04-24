#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;



#define M1I1 9
#define M1I2 8
#define M2I1 7
#define M2I2 6

#define M1_PWM 10
#define M2_PWM 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
void motor1_action(int move_indicator, int pwm);
void motor2_action(int move_indicator, int pwm);
void stop();

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void twist_callback(const void * msgin) 
{
  
  pub_msg.data = 1;
  rcl_publish(&publisher, &pub_msg, NULL);
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  printf("Received: %f\n", msg->linear.x);
// forward
    if(msg->linear.x > 0.4 && msg->angular.z < 0.1 && msg->angular.z > -0.1){
        motor1_action(1, 10000);
        motor2_action(1, 10000);
    }
    //backward
    else if(msg->linear.x < -0.4 && msg->angular.z < 0.1 && msg->angular.z > -0.1){
        motor1_action(0, 10000);
        motor2_action(0, 10000);
    }
    //left
    else if(msg->linear.x < 0.1 && msg->linear.x > -0.1 && msg->angular.z > 0.48){
        motor1_action(0, 10000);
        motor2_action(1, 10000);
    }
    //right
    else if(msg->linear.x < 0.1 && msg->linear.x > -0.1 && msg->angular.z < -0.48){
        motor1_action(1, 10000);
        motor2_action(0, 10000);
    }
    //forward left
    else if(msg->linear.x < 0.4  && msg->linear.x > 0.1 && msg->angular.z > 0.0){
        motor1_action(1, 1000);
        motor2_action(1, 10000);
    }

    //forward right
    else if(msg->linear.x < 0.4  && msg->linear.x > 0.1 && msg->angular.z < 0.0){
        motor1_action(1, 10000);
        motor2_action(1, 1000);
    }

    //backward left
    else if(msg->linear.x > -0.4  && msg->linear.x < -0.1 && msg->angular.z > 0.0){
        motor1_action(0, 1000);
        motor2_action(0, 10000);
    }
    //backward right
    else if(msg->linear.x > -0.4  && msg->linear.x < -0.1 && msg->angular.z < 0.0){
        motor1_action(0, 10000);
        motor2_action(0, 1000);
    }

    //stop
    else {
        stop();
    }
}

void motor1_action(int move_indicator, int pwm){
    analogWrite(M1_PWM, pwm);
    if(move_indicator > 0){
      digitalWrite(M1I1, HIGH);
      digitalWrite(M1I2, LOW);
    }else{
      digitalWrite(M1I1, LOW);
      digitalWrite(M1I2, HIGH);
    }
}

void motor2_action(int move_indicator, int pwm){
    analogWrite(M2_PWM, pwm);
    if(move_indicator > 0){
      digitalWrite(M2I1, HIGH);
      digitalWrite(M2I2, LOW);
    }else{
      digitalWrite(M2I1, LOW);
      digitalWrite(M2I2, HIGH);
    }
}

void stop(){
      digitalWrite(M2I1, LOW);
      digitalWrite(M2I2, LOW);
      digitalWrite(M2I1, LOW);
      digitalWrite(M2I2, LOW);
       analogWrite(M1_PWM, 0);
       analogWrite(M2_PWM, 0);
}



void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);


  pinMode(M1I1, OUTPUT);
  pinMode(M1I2, OUTPUT);
  pinMode(M2I1, OUTPUT);
  pinMode(M2I2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  analogWrite(M1_PWM, abs(0));
  analogWrite(M2_PWM, abs(0));


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //msg.data = 0;
  RCCHECK(rclc_subscription_init_default( 
        &subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
  
  RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &msg,
        &twist_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
