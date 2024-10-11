#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
rcl_publisher_t c_publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Float32MultiArray pub_msg;
std_msgs__msg__Float32MultiArray c_pub_msg;
std_msgs__msg__Float32MultiArray sup_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include <Adafruit_BNO08x.h>
#include <utility/imumaths.h>

#include "Mobile_command.h"
#include "math.h"

// Print loop Time (100 Hz)
unsigned long prev_timestep_print;
unsigned long current_timestep_print;
unsigned long timestamp_print = 0;
int32_t timestep_print = 10000;
// Control loop Time (1000 Hz)
unsigned long prev_timestep;
unsigned long current_timestep;
unsigned long timestamp = 0;
int timestep = 1000;

float vx, vy, vw = 0;

uint8_t flag = 0;
Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
IMU_DATA imu_data;
ODOM_DATA odom_data;
ODOM_DATA odom_cmd;
// SCL yellow
TaskHandle_t Task1;
TaskHandle_t Task2;
///Sensor set
void setReports()
{
   Serial.println("Setting desired reports");

  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
    Serial.println("Could not enable step counter");
  }
  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
    Serial.println("Could not enable stability classifier");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
    Serial.println("Could not enable shake detector");
  }
  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
    Serial.println("Could not enable personal activity classifier");
  }
}
// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    // imu_data = Mobile.getIMU();
    odom_data = Mobile.getODOM();

    pub_msg.data.data[3] = Mobile.fb_qd[0]; // qd0
    pub_msg.data.data[4] = Mobile.fb_qd[1]; // qd1
    pub_msg.data.data[5] = Mobile.fb_qd[2]; // qd2
    pub_msg.data.data[6] = Mobile.fb_qd[3]; // qd3

    // Populate data
    pub_msg.data.data[7] = odom_data.vx; // odom_Vx
    pub_msg.data.data[8] = odom_data.vy; // odom_Vy
    pub_msg.data.data[9] = odom_data.wz; // odom_Vw

    // // Populate data
    pub_msg.data.data[10] = imu_data.gyro.x; // gyro x
    pub_msg.data.data[11] = imu_data.gyro.y; // gyro y
    pub_msg.data.data[12] = imu_data.gyro.z; // gyro z`

    pub_msg.data.data[13] = imu_data.accel.x; // accel x
    pub_msg.data.data[14] = imu_data.accel.y; // accel y
    pub_msg.data.data[15] = imu_data.accel.z; // accel z

    pub_msg.data.data[16] = imu_data.quat.x; // quat x
    pub_msg.data.data[17] = imu_data.quat.y; // quat y
    pub_msg.data.data[18] = imu_data.quat.z; // quat z
    pub_msg.data.data[19] = imu_data.quat.w; // quat w

    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}

void subscription_callback(const void *msgin)
{
  // neopixelWrite(21, 0, 10, 0);
  // delay(10);
  const std_msgs__msg__Float32MultiArray *float_msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  c_pub_msg.data.data[0] = float_msg->data.data[0];
  c_pub_msg.data.data[1] = float_msg->data.data[1];
  c_pub_msg.data.data[2] = float_msg->data.data[2];

  odom_cmd.vx = float_msg->data.data[0];
  odom_cmd.vy = float_msg->data.data[1];
  odom_cmd.wz = float_msg->data.data[2];
  // neopixelWrite(21, 0, 0, 10);
  // delay(10);
}

// Task1code: blinks an LED every 1000 ms
void Task1code(void *pvParameters)
{
  // neopixelWrite(21, 0, 10, 0);
  // Serial.begin(921600);
  Serial.println("test task1");
  set_microros_serial_transports(Serial);
  // delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "my_response_topic"));

  RCCHECK(rclc_publisher_init_default(
      &c_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "c_my_response"));

  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "float_dumb_topic"));

  sup_msg.data.capacity = 10;
  sup_msg.data.size = 0;
  sup_msg.data.data = (float *)malloc(sup_msg.data.capacity * sizeof(float));

  sup_msg.layout.dim.capacity = 10;
  sup_msg.layout.dim.size = 0;
  sup_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(sup_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < sup_msg.layout.dim.capacity; i++)
  {
    sup_msg.layout.dim.data[i].label.capacity = 10;
    sup_msg.layout.dim.data[i].label.size = 0;
    sup_msg.layout.dim.data[i].label.data = (char *)malloc(sup_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create timer,
  // const unsigned int timer_timeout = 10;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sup_msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_subscription);

  pub_msg.data.capacity = 13;
  pub_msg.data.size = 13;
  pub_msg.data.data = (float *)malloc(pub_msg.data.capacity * sizeof(float));

  // Populate data
  pub_msg.data.data[0] = 0.0f; // Vx
  pub_msg.data.data[1] = 0.0f; // Vy
  pub_msg.data.data[2] = 0.0f; // Wz

  pub_msg.data.data[3] = 0.0f; // gyro x
  pub_msg.data.data[4] = 0.0f; // gyro y
  pub_msg.data.data[5] = 0.0f; // gyro z

  pub_msg.data.data[6] = 0.0f; // accel x
  pub_msg.data.data[7] = 0.0f; // accel y
  pub_msg.data.data[8] = 0.0f; // accel z

  pub_msg.data.data[9] = 0.0f;  // quat x
  pub_msg.data.data[10] = 0.0f; // quat y
  pub_msg.data.data[11] = 0.0f; // quat z
  pub_msg.data.data[12] = 0.0f; // quat w

  c_pub_msg.data.capacity = 5;
  c_pub_msg.data.size = 5;
  c_pub_msg.data.data = (float *)malloc(c_pub_msg.data.capacity * sizeof(float));
  pub_msg.data.data[0] = 0.0f;
  pub_msg.data.data[1] = 0.0f;
  pub_msg.data.data[2] = 0.0f;
  pub_msg.data.data[3] = 0.0f;
  pub_msg.data.data[4] = 0.0f;

  // neopixelWrite(21, 0, 0, 10);

  for (;;)
  {
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    current_timestep_print = micros();
    if (current_timestep_print - timestamp_print > timestep_print)
    {
      c_pub_msg.data.data[3] = current_timestep_print - timestamp_print;
      timestamp_print = micros();
      // imu_data = Mobile.getIMU();
      odom_data = Mobile.getODOM();

      // pub_msg.data.data[0] = odom_data.vx; //Vx
      pub_msg.data.data[0] = odom_data.vx; // Vx
      pub_msg.data.data[1] = odom_data.vy; // Vy
      pub_msg.data.data[2] = odom_data.wz; // Wz

      pub_msg.data.data[3] = imu_data.gyro.x; // gyro x
      pub_msg.data.data[4] = imu_data.gyro.y; // gyro y
      pub_msg.data.data[5] = imu_data.gyro.z; // gyro z

      pub_msg.data.data[6] = imu_data.accel.x; // accel x
      pub_msg.data.data[7] = imu_data.accel.y; // accel y
      pub_msg.data.data[8] = imu_data.accel.z; // accel z

      pub_msg.data.data[9] = imu_data.quat.x;  // quat x
      pub_msg.data.data[10] = imu_data.quat.y; // quat y
      pub_msg.data.data[11] = imu_data.quat.z; // quat z
      pub_msg.data.data[12] = imu_data.quat.w; // quat w

      RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    }
    if (current_timestep_print - timestamp_print > timestep_print)
    {
      RCSOFTCHECK(rcl_publish(&c_publisher, &c_pub_msg, NULL));
    }
  }
}

// Task2code: blinks an LED every 700 ms
void Task2code(void *pvParameters)
{
  Serial.begin(115200);
  Wire.begin(15, 16);
  while (!Serial)
  {
    delay(10); // Wait until serial console opens
  }
  // Serial.println("test task2");
  // delay(5000);
  neopixelWrite(21, 10, 0, 0);
  if (!bno08x.begin_I2C(0x4A, &Wire))
  {
    neopixelWrite(21, 0, 10, 0);
  }
  setReports();
  // Mobile.begin();
  // delay(5000);
  // neopixelWrite(21, 0, 0, 10);
  for (;;)
  {
    // neopixelWrite(21, 0, 0, 10);
    if (bno08x.getSensorEvent(&sensorValue))
    {
      neopixelWrite(21, 0, 0, 10);
      switch (sensorValue.sensorId)
      {
      case SH2_GYROSCOPE_CALIBRATED:
        neopixelWrite(21, 10, 0, 0);
        imu_data.gyro.x = sensorValue.un.gyroscope.x;
        imu_data.gyro.y = sensorValue.un.gyroscope.y;
        imu_data.gyro.z = sensorValue.un.gyroscope.z;
        break;
      case SH2_LINEAR_ACCELERATION:
        neopixelWrite(21, 10, 0, 0);
        imu_data.accel.x = sensorValue.un.linearAcceleration.x;
        imu_data.accel.y = sensorValue.un.linearAcceleration.y;
        imu_data.accel.z = sensorValue.un.linearAcceleration.z;
        break;
      case SH2_ROTATION_VECTOR:
        neopixelWrite(21, 10, 0, 0);
        imu_data.quat.x = sensorValue.un.rotationVector.i;
        imu_data.quat.y = sensorValue.un.rotationVector.j;
        imu_data.quat.z = sensorValue.un.rotationVector.k;
        imu_data.quat.w = sensorValue.un.rotationVector.real;
        break;
      }
    }
    else
    {
      neopixelWrite(21, 0, 10, 0);
    }

    current_timestep = micros();
    if (current_timestep - timestamp > timestep)
    {
      // c_pub_msg.data.data[4] = current_timestep - timestamp;
      timestamp = micros();
      // Serial.begin(921600);
      // Serial.print(encx[0]->get_diff_count());
      // Serial.print(" ");
      // Serial.print(encx[1]->get_diff_count());
      // Serial.print(" ");
      // Serial.print(encx[2]->get_diff_count());
      // Serial.print(" ");
      // Serial.println(encx[3]->get_diff_count());
      Mobile.control(odom_cmd.vx, odom_cmd.vy, odom_cmd.wz);
      // Mobile.control(0, 0.25, 0);
      // Serial.println("run");
    }
  }
}

void setup()
{
  // Configure serial transport

  // neopixelWrite(21, 0, 0, 10);
  // delay(5000);
  // Initialize I2C with custom SDA and SCL pins
  // Serial.println("Hello wwwwwwwwwwwwwwwwwwwwww");

  // Serial.println("Adafruit BNO08x test!");
  // Initialize BNO08x
  // Wire.setClock(100000);  // Set I2C bus speed to 100 kHz
  // neopixelWrite(21, 0, 10, 0);
  // delay(5000);
  // if (!bno08x.begin_I2C(0x4A, &Wire)) {
  // Serial.println("Failed to find BNO08x chip");
  // while (1) {
  //   neopixelWrite(21, 0, 0, 10);
  //   delay(10);
  // }
  // }
  // Serial.println("BNO08x Found!");
  Mobile.begin();
  // delay(5000);
  // neopixelWrite(21, 10, 0, 0);
  // Serial.println("test setup");
  // neopixelWrite(21, 0, 0, 10);
  delay(500);
  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
  delay(500);

  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
  delay(500);
}

void loop()
{
  // Serial.println("test setup");
  delay(1);
}