#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <ESP32Servo.h>

// --- PIN TANIMLAMALARI ---
#define PIN_PRISMATIC 13  
#define PIN_SHOULDER  12  
#define PIN_ELBOW     14  

// --- AYARLAR ---
#define MIN_PULSE 500
#define MAX_PULSE 2500
#define THRESHOLD 2   // 2 Dereceden az degisimde motoru oynatma (Titremeyi onler)

// --- SERVO NESNELERI ---
Servo servo_prismatic;
Servo servo_shoulder;
Servo servo_elbow;

// Son pozisyonlari hafizada tutmak icin
int last_angle_prismatic = -1;
int last_angle_shoulder = -1;
int last_angle_elbow = -1;

// --- ROS NESNELERI ---
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Kontrol Bayragi
bool is_attached = false;

float rad2deg(float rad) {
  return rad * 180.0 / 3.14159;
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(2, !digitalRead(2)); 
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

  // ILK BAGLANTI: Veri geldigi an servolari aktif et
  if (!is_attached) {
      servo_prismatic.setPeriodHertz(50);
      servo_prismatic.attach(PIN_PRISMATIC, MIN_PULSE, MAX_PULSE);

      servo_shoulder.setPeriodHertz(50);
      servo_shoulder.attach(PIN_SHOULDER, MIN_PULSE, MAX_PULSE);

      servo_elbow.setPeriodHertz(50);
      servo_elbow.attach(PIN_ELBOW, MIN_PULSE, MAX_PULSE);
      
      is_attached = true;
  }

  // Veri geldigini gormek icin LED'i cok kisa yakip sondur
  digitalWrite(2, HIGH);

  for(size_t i = 0; i < msg->name.size; i++){
    String joint_name = String(msg->name.data[i].data);
    float position = msg->position.data[i];
    int target_angle = 90;

    // 1. ASANSOR
    if(joint_name == "base_shoulder_joint"){
       if(position < 0) position = 0;
       target_angle = (int)(position * (180.0 / 0.14));
       target_angle = constrain(target_angle, 0, 180);
       
       // Sadece fark THRESHOLD'dan buyukse hareket et
       if(abs(target_angle - last_angle_prismatic) > THRESHOLD) {
           servo_prismatic.write(target_angle);
           last_angle_prismatic = target_angle;
       }
    }

    // 2. OMUZ
    else if(joint_name == "arm_joint"){
       target_angle = (int)rad2deg(position) + 90;
       target_angle = constrain(target_angle, 0, 180);
       
       if(abs(target_angle - last_angle_shoulder) > THRESHOLD) {
           servo_shoulder.write(target_angle);
           last_angle_shoulder = target_angle;
       }
    }

    // 3. DIRSEK
    else if(joint_name == "joint3"){
       target_angle = (int)rad2deg(position) + 90;
       target_angle = constrain(target_angle, 0, 180);
       
       if(abs(target_angle - last_angle_elbow) > THRESHOLD) {
           servo_elbow.write(target_angle);
           last_angle_elbow = target_angle;
       }
    }
  }
  digitalWrite(2, LOW); // LED sondur
}

void setup() {
  set_microros_transports(); 
  pinMode(2, OUTPUT); // Dahili LED
  
  // Baslangicta servolari pin mode olarak kapa
  pinMode(PIN_PRISMATIC, OUTPUT);
  pinMode(PIN_SHOULDER, OUTPUT);
  pinMode(PIN_ELBOW, OUTPUT);
  digitalWrite(PIN_PRISMATIC, LOW);
  digitalWrite(PIN_SHOULDER, LOW);
  digitalWrite(PIN_ELBOW, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_stable_servo", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  static double memory_position[10];
  static double memory_velocity[10];
  static double memory_effort[10];
  static rosidl_runtime_c__String memory_name[10];
  
  for(int i=0; i<10; i++){
      memory_name[i].data = (char*)malloc(50 * sizeof(char));
      memory_name[i].size = 0;
      memory_name[i].capacity = 50;
  }
  msg.position.data = memory_position; msg.position.capacity = 10; msg.position.size = 0;
  msg.velocity.data = memory_velocity; msg.velocity.capacity = 10; msg.velocity.size = 0;
  msg.effort.data = memory_effort;     msg.effort.capacity = 10;   msg.effort.size = 0;
  msg.name.data = memory_name;         msg.name.capacity = 10;     msg.name.size = 0;

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}