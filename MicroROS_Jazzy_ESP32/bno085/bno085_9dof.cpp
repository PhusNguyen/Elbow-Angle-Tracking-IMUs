#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define IMU1_ADDR 0x4B
#define IMU2_ADDR 0x4A

// Global parameters
unsigned long sample_count = 0;
unsigned long boot_time = 0;
unsigned long last_report_time = 0;

// Declare instances for imus
BNO080 bno1;
BNO080 bno2;

// IMU publishers (accel + gyro)
rcl_publisher_t imu1_publisher;
rcl_publisher_t imu2_publisher;

// Magnetometer publishers
rcl_publisher_t mag1_publisher;
rcl_publisher_t mag2_publisher;

sensor_msgs__msg__Imu imu1_msg;
sensor_msgs__msg__Imu imu2_msg;
sensor_msgs__msg__MagneticField mag1_msg;
sensor_msgs__msg__MagneticField mag2_msg;

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

  // No orientation provided — set covariance[0] to -1 per REP-145
  msg->orientation_covariance[0] = -1.0;
}

void init_mag_msg(sensor_msgs__msg__MagneticField *msg, const char *frame_id) {
  msg->header.frame_id.data = (char*)frame_id;
  msg->header.frame_id.size = strlen(frame_id);
  msg->header.frame_id.capacity = strlen(frame_id) + 1;
}

void fill_and_publish(
  rcl_publisher_t *imu_pub,
  rcl_publisher_t *mag_pub,
  sensor_msgs__msg__Imu *imu_msg,
  sensor_msgs__msg__MagneticField *mag_msg,
  BNO080 *bno,
  int64_t stamp_ns
) {
  int32_t sec = stamp_ns / 1000000000;
  uint32_t nsec = stamp_ns % 1000000000;

  // IMU messages (accel + gyro)
  imu_msg->header.stamp.sec = sec;
  imu_msg->header.stamp.nanosec = nsec;

  imu_msg->linear_acceleration.x = bno->getAccelX();
  imu_msg->linear_acceleration.y = bno->getAccelY();
  imu_msg->linear_acceleration.z = bno->getAccelZ();

  imu_msg->angular_velocity.x = bno->getGyroX();
  imu_msg->angular_velocity.y = bno->getGyroY();
  imu_msg->angular_velocity.z = bno->getGyroZ();

  rcl_publish(imu_pub, imu_msg, NULL);

  // Magnetometer messages
  // BNO085 returns mag in uT, ROS MagneticField expects Tesla
  mag_msg->header.stamp.sec = sec;
  mag_msg->header.stamp.nanosec = nsec;

  mag_msg->magnetic_field.x = bno->getMagX() * 1e-6;
  mag_msg->magnetic_field.y = bno->getMagY() * 1e-6;
  mag_msg->magnetic_field.z = bno->getMagZ() * 1e-6;

  rcl_publish(mag_pub, mag_msg, NULL);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  if (bno1.dataAvailable()) {
    int64_t t1 = rmw_uros_epoch_nanos();
    fill_and_publish(&imu1_publisher, &mag1_publisher,
                     &imu1_msg, &mag1_msg, &bno1, t1);
    sample_count++;
  }

  if (bno2.dataAvailable()) {
    int64_t t2 = rmw_uros_epoch_nanos();
    fill_and_publish(&imu2_publisher, &mag2_publisher,
                     &imu2_msg, &mag2_msg, &bno2, t2);
  }
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu1_publisher, &node);
  rcl_publisher_fini(&imu2_publisher, &node);
  rcl_publisher_fini(&mag1_publisher, &node);
  rcl_publisher_fini(&mag2_publisher, &node);
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

  // IMU publishers
  RCCHECK(rclc_publisher_init_default(
    &imu1_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu1/data_raw"));

  RCCHECK(rclc_publisher_init_default(
    &imu2_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu2/data_raw"));

  // Magnetometer publishers
  RCCHECK(rclc_publisher_init_default(
    &mag1_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu1/mag"));

  RCCHECK(rclc_publisher_init_default(
    &mag2_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu2/mag"));

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
  if (!bno1.begin(IMU1_ADDR, Wire1)) {
    error_blink(100, 100);
  }

  // IMU2 init — medium blink (250ms) on failure
  if (!bno2.begin(IMU2_ADDR, Wire1)) {
    error_blink(250, 250);
  }

  // Enable sensors reading rate
  bno1.enableAccelerometer(25);
  bno1.enableGyro(25);
  bno1.enableMagnetometer(25);
  bno2.enableAccelerometer(25);
  bno2.enableGyro(25);
  bno2.enableMagnetometer(25);

  init_imu_msg(&imu1_msg, "imu1_frame");
  init_imu_msg(&imu2_msg, "imu2_frame");
  init_mag_msg(&mag1_msg, "imu1_frame");
  init_mag_msg(&mag2_msg, "imu2_frame");

  // state init
  state = WAITING_FOR_AGENT;

  // start timer for health check
  boot_time = millis();
  last_report_time = millis();
}

void loop() {
  switch (state) {

    case WAITING_FOR_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // check to change this led mode
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

      // Health check every 1 second
      {
        unsigned long now = millis();
        if (now - last_report_time >= 1000) {
          if ((now - boot_time > 4000) && sample_count < 10) {
            esp_restart();
          }
          sample_count = 0;
          last_report_time = now;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      digitalWrite(LED_BUILTIN, HIGH);
      destroy_entities();
      state = WAITING_FOR_AGENT;
      break;
  }
}
