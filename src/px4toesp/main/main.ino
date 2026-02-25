#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <ESP32Servo.h>

// ─── PINES ────────────────────────────────────────────────────────────────────
#define PIN_M1    12
#define PIN_M2    14
#define PIN_M3    27
#define PIN_M4    25
#define LED_PIN    2

// ─── PWM ──────────────────────────────────────────────────────────────────────
#define PWM_FREQ     200    
#define PWM_MIN      1100
#define PWM_MID      1500
#define PWM_MAX      1900
#define MSG_TIMEOUT  1000

Servo motor1, motor2, motor3, motor4;

uint16_t target_m1 = PWM_MID;
uint16_t target_m2 = PWM_MID;
uint16_t target_m3 = PWM_MID;
uint16_t target_m4 = PWM_MID;

rcl_subscription_t sub_pwm;
std_msgs__msg__UInt16MultiArray msg_pwm;
uint16_t pwm_data[4];

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

unsigned long last_msg_time = 0;
bool ros_active = false;
bool connected  = false;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) return false; }

// ─────────────────────────────────────────────────────────────────────────────
void writeTargets() {
  motor1.writeMicroseconds(target_m1);
  motor2.writeMicroseconds(target_m2);
  motor3.writeMicroseconds(target_m3);
  motor4.writeMicroseconds(target_m4);
}

void setAllNeutral() {
  target_m1 = PWM_MID;
  target_m2 = PWM_MID;
  target_m3 = PWM_MID;
  target_m4 = PWM_MID;
}

void cb_pwm(const void* msgin) {
  const std_msgs__msg__UInt16MultiArray* msg =
    (const std_msgs__msg__UInt16MultiArray*)msgin;

  if (msg->data.size >= 4) {
    target_m1 = constrain(msg->data.data[0], PWM_MIN, PWM_MAX);
    target_m2 = constrain(msg->data.data[1], PWM_MIN, PWM_MAX);
    target_m3 = constrain(msg->data.data[2], PWM_MIN, PWM_MAX);
    target_m4 = constrain(msg->data.data[3], PWM_MIN, PWM_MAX);

    last_msg_time = millis();
    if (!ros_active) {
      ros_active = true;
      digitalWrite(LED_PIN, HIGH);
    }
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_thruster_controller", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &sub_pwm, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/pwm_outputs"
  ));

  msg_pwm.data.data     = pwm_data;
  msg_pwm.data.size     = 4;
  msg_pwm.data.capacity = 4;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub_pwm, &msg_pwm, &cb_pwm, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rclc_executor_fini(&executor);
  rcl_subscription_fini(&sub_pwm, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  motor1.setPeriodHertz(PWM_FREQ);
  motor1.attach(PIN_M1, PWM_MIN, PWM_MAX);
  motor1.writeMicroseconds(PWM_MID);

  motor2.setPeriodHertz(PWM_FREQ);
  motor2.attach(PIN_M2, PWM_MIN, PWM_MAX);
  motor2.writeMicroseconds(PWM_MID);

  motor3.setPeriodHertz(PWM_FREQ);
  motor3.attach(PIN_M3, PWM_MIN, PWM_MAX);
  motor3.writeMicroseconds(PWM_MID);

  motor4.setPeriodHertz(PWM_FREQ);
  motor4.attach(PIN_M4, PWM_MIN, PWM_MAX);
  motor4.writeMicroseconds(PWM_MID);

  delay(5000);

  set_microros_serial_transports(Serial);

  motor1.writeMicroseconds(PWM_MID);
  motor2.writeMicroseconds(PWM_MID);
  motor3.writeMicroseconds(PWM_MID);
  motor4.writeMicroseconds(PWM_MID);

  last_msg_time = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  writeTargets();

  if ((millis() - last_msg_time) > MSG_TIMEOUT) {
    if (ros_active) {
      ros_active = false;
      digitalWrite(LED_PIN, LOW);
    }
    setAllNeutral();
  }

  if (!connected) {
    if (create_entities()) {
      connected = true;
      last_msg_time = millis();
    } else {
      destroy_entities();
    }
  } else {
    if (RCL_RET_OK != rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))) {
      connected = false;
      destroy_entities();
    }
  }

  delay(10);
}
