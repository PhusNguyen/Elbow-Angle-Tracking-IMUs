#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>

#define IMU1_ADDR 0x69
#define IMU2_ADDR 0x68

Adafruit_ICM20649 icm1; // SDO pin HIGH  -> address 0x69
Adafruit_ICM20649 icm2; // SDO pin LOW -> address 0x68

rcl_publisher_t imu1_publisher;
rcl_publisher_t imu2_publisher;

sensor_msgs__msg__Imu imu1_msg;
sensor_msgs__msg__Imu imu2_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile unsigned long prev = 0; \
  unsigned long curr = millis(); \
  if (curr - prev >= MS) { prev = curr; X; } \
} while(0)

enum State {
  WAITING_FOR_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// LED blink patterns for different error states
void error_blink(int on_ms, int off_ms) {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(on_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(off_ms);
  }
}

void init_imu_msg(sensor_msgs__msg__Imu *msg, const char *frame_id) {
  msg->header.frame_id.data = (char*)frame_id;
  msg->header.frame_id.size = strlen(frame_id);
  msg->header.frame_id.capacity = strlen(frame_id) + 1;

  // No orientation estimate — set covariance[0] to -1 per REP-145
  msg->orientation.w = 0.0;
  msg->orientation.x = 0.0;
  msg->orientation.y = 0.0;
  msg->orientation.z = 0.0;
  msg->orientation_covariance[0] = -1.0;
}

void fill_and_publish(
  rcl_publisher_t *pub,
  sensor_msgs__msg__Imu *msg,
  sensors_event_t *accel,
  sensors_event_t *gyro,
  int64_t stamp_ns
) {
  msg->header.stamp.sec = stamp_ns / 1000000000;
  msg->header.stamp.nanosec = stamp_ns % 1000000000;

  msg->linear_acceleration.x = accel->acceleration.x;
  msg->linear_acceleration.y = accel->acceleration.y;
  msg->linear_acceleration.z = accel->acceleration.z;

  msg->angular_velocity.x = gyro->gyro.x;
  msg->angular_velocity.y = gyro->gyro.y;
  msg->angular_velocity.z = gyro->gyro.z;

  rcl_publish(pub, msg, NULL);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  sensors_event_t accel1, gyro1, temp1;
  sensors_event_t accel2, gyro2, temp2;

  icm1.getEvent(&accel1, &gyro1, &temp1);
  int64_t t1 = rmw_uros_epoch_nanos();
  fill_and_publish(&imu1_publisher, &imu1_msg, &accel1, &gyro1, t1);

  icm2.getEvent(&accel2, &gyro2, &temp2);
  int64_t t2 = rmw_uros_epoch_nanos();
  fill_and_publish(&imu2_publisher, &imu2_msg, &accel2, &gyro2, t2);
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu1_publisher, &node);
  rcl_publisher_fini(&imu2_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Sync time with ROS2 agent
  RCCHECK(rmw_uros_sync_session(1000));

  RCCHECK(rclc_node_init_default(&node, "dual_imu_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &imu1_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu1/data_raw"));

  RCCHECK(rclc_publisher_init_default(
    &imu2_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu2/data_raw"));

  // 10ms = 100 Hz timer
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(25),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  Wire1.begin();
  Wire1.setClock(400000);

  // IMU1 init — fast blink (100ms) on failure
  if (!icm1.begin_I2C(IMU1_ADDR, &Wire1)) {
    error_blink(100, 100);
  }

  // IMU2 init — medium blink (250ms) on failure
  if (!icm2.begin_I2C(IMU2_ADDR, &Wire1)) {
    error_blink(250, 250);
  }

  // Configure both sensors identically
  icm1.setAccelRange(ICM20649_ACCEL_RANGE_8_G);
  icm2.setAccelRange(ICM20649_ACCEL_RANGE_8_G);
  icm1.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
  icm2.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);

  // Internal rate >= output rate
  icm1.setAccelRateDivisor(25);
  icm2.setAccelRateDivisor(25);
  icm1.setGyroRateDivisor(25);
  icm2.setGyroRateDivisor(25);

  init_imu_msg(&imu1_msg, "imu1_frame");
  init_imu_msg(&imu2_msg, "imu2_frame");

  state = WAITING_FOR_AGENT;
}

void loop() {
  switch (state) {

    case WAITING_FOR_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        state = (RMW_RET_OK == rmw_uros_ping_agent(10, 1))
          ? AGENT_AVAILABLE : WAITING_FOR_AGENT;
      });
      break;

    case AGENT_AVAILABLE:
      digitalWrite(LED_BUILTIN, HIGH);
      state = create_entities() ? AGENT_CONNECTED : WAITING_FOR_AGENT;
      break;

    case AGENT_CONNECTED:
      digitalWrite(LED_BUILTIN, LOW);

      // Let executor handle timer scheduling directly
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

      EXECUTE_EVERY_N_MS(200, {
        if (RMW_RET_OK != rmw_uros_ping_agent(10, 1)) {
          state = AGENT_DISCONNECTED;
        }
      });
      break;

    case AGENT_DISCONNECTED:
      digitalWrite(LED_BUILTIN, HIGH);
      destroy_entities();
      state = WAITING_FOR_AGENT;
      break;
  }
}