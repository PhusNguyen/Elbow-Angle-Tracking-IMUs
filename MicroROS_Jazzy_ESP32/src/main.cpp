#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <SparkFun_BNO080_Arduino_Library.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// IMU Configuration
#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define IMU1_ADDR 0x4B // Upperarm 
#define IMU2_ADDR 0x4A // Forearm 

// Initialize sensor classes
BNO080 myIMU1;
BNO080 myIMU2;

// ROS publishers
rcl_publisher_t imu1_publisher;
rcl_publisher_t imu2_publisher;
rcl_publisher_t mag1_publisher;
rcl_publisher_t mag2_publisher;

// ROS messages
sensor_msgs__msg__Imu imu1_msg;
sensor_msgs__msg__Imu imu2_msg;
sensor_msgs__msg__MagneticField mag1_msg;
sensor_msgs__msg__MagneticField mag2_msg;

// ROS infrastructure
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void init_imu_messages(){
  // Initialize IMU1 message
  imu1_msg.header.frame_id.data = (char*)"imu1_frame";

  // Initialize IMU2 message
  imu2_msg.header.frame_id.data = (char*)"imu2_frame";

  // Initialize Mag1 message
  mag1_msg.header.frame_id.data = (char*)"imu1_frame";

  // Initialize Mag2 message
  mag2_msg.header.frame_id.data = (char*)"imu2_frame";
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Get current timestamp
    unsigned long now = millis();
    
    // Read and publish IMU1 data
    if (myIMU1.dataAvailable()) {
      // Update timestamp
      imu1_msg.header.stamp.sec = now / 1000;
      imu1_msg.header.stamp.nanosec = (now % 1000) * 1000000;
      mag1_msg.header.stamp = imu1_msg.header.stamp;
      
      // Get acceleration data
      imu1_msg.linear_acceleration.x = myIMU1.getAccelX();
      imu1_msg.linear_acceleration.y = myIMU1.getAccelY();
      imu1_msg.linear_acceleration.z = myIMU1.getAccelZ();
      
      // Get gyroscope data (angular velocity in rad/s)
      imu1_msg.angular_velocity.x = myIMU1.getGyroX();
      imu1_msg.angular_velocity.y = myIMU1.getGyroY();
      imu1_msg.angular_velocity.z = myIMU1.getGyroZ();
      
      // Get magnetometer data (in microTesla)
      mag1_msg.magnetic_field.x = myIMU1.getMagX();
      mag1_msg.magnetic_field.y = myIMU1.getMagY();
      mag1_msg.magnetic_field.z = myIMU1.getMagZ();
      
      // Note: BNO080 doesn't provide orientation directly in this mode
      // You would need to enable rotation vector for quaternion data
      // For now, setting orientation to identity quaternion
      imu1_msg.orientation.x = myIMU1.getQuatI();
      imu1_msg.orientation.y = myIMU1.getQuatJ();
      imu1_msg.orientation.z = myIMU1.getQuatK();
      imu1_msg.orientation.w = myIMU1.getQuatReal();
      
      // Publish IMU1 data
      RCSOFTCHECK(rcl_publish(&imu1_publisher, &imu1_msg, NULL));
      RCSOFTCHECK(rcl_publish(&mag1_publisher, &mag1_msg, NULL));
    }
    
    // Read and publish IMU2 data
    if (myIMU2.dataAvailable()) {
      // Update timestamp
      imu2_msg.header.stamp.sec = now / 1000;
      imu2_msg.header.stamp.nanosec = (now % 1000) * 1000000;
      mag2_msg.header.stamp = imu2_msg.header.stamp;
      
      // Get acceleration data (using linear acceleration for IMU2)
      imu2_msg.linear_acceleration.x = myIMU2.getAccelX();
      imu2_msg.linear_acceleration.y = myIMU2.getAccelY();
      imu2_msg.linear_acceleration.z = myIMU2.getAccelZ();
      
      // Get gyroscope data (angular velocity in rad/s)
      imu2_msg.angular_velocity.x = myIMU2.getGyroX();
      imu2_msg.angular_velocity.y = myIMU2.getGyroY();
      imu2_msg.angular_velocity.z = myIMU2.getGyroZ();
      
      // Get magnetometer data (in microTesla)
      mag2_msg.magnetic_field.x = myIMU2.getMagX();
      mag2_msg.magnetic_field.y = myIMU2.getMagY();
      mag2_msg.magnetic_field.z = myIMU2.getMagZ();
      
      // Note: BNO080 doesn't provide orientation directly in this mode
      imu2_msg.orientation.x = myIMU2.getQuatI();
      imu2_msg.orientation.y = myIMU2.getQuatJ();
      imu2_msg.orientation.z = myIMU2.getQuatK();
      imu2_msg.orientation.w = myIMU2.getQuatReal();
      
      // Publish IMU2 data
      RCSOFTCHECK(rcl_publish(&imu2_publisher, &imu2_msg, NULL));
      RCSOFTCHECK(rcl_publish(&mag2_publisher, &mag2_msg, NULL));
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configure serial transport for microROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  // Initialize I2C for IMUs
  Wire1.begin();
  Wire1.setClock(400000); // Increase I2C data rate to 400kHz
  
  // Initialize IMU1
  if (myIMU1.begin(IMU1_ADDR, Wire1) == false) {
    // Flash LED rapidly if IMU1 not found
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(50);
    }
  }
  
  // Initialize IMU2
  if (myIMU2.begin(IMU2_ADDR, Wire1) == false) {
    // Flash LED rapidly if IMU2 not found
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
  
  const unsigned int sensor_reading_rate = 20;
  // Configure IMU1 sensors
  myIMU1.enableAccelerometer(sensor_reading_rate);  
  myIMU1.enableGyro(sensor_reading_rate);
  myIMU1.enableMagnetometer(sensor_reading_rate);
  myIMU1.enableGameRotationVector(sensor_reading_rate);
  
  // Configure IMU2 sensors  
  myIMU2.enableAccelerometer(sensor_reading_rate);  // Linear acceleration (gravity removed)
  myIMU2.enableGyro(sensor_reading_rate);
  myIMU2.enableMagnetometer(sensor_reading_rate);
  myIMU2.enableGameRotationVector(sensor_reading_rate);
  
  // Initialize microROS
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "dual_imu_node", "", &support));
  
  // Create publishers for IMU1
  RCCHECK(rclc_publisher_init_default(
    &imu1_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu1/data"));
    
  RCCHECK(rclc_publisher_init_default(
    &mag1_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu1/mag"));
  
  // Create publishers for IMU2
  RCCHECK(rclc_publisher_init_default(
    &imu2_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu2/data"));
    
  RCCHECK(rclc_publisher_init_default(
    &mag2_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu2/mag"));
  
  // Create timer
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Initialize message structures
  init_imu_messages();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

