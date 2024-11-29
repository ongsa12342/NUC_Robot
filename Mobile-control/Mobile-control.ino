#include "Mobile_command.h"
void timer0_callback();
void control_loop(void *pv);

//Control loop Time (1 Hz)
unsigned long timestamp_cmd = 0;
int timestep_cmd = 2.5e6;

float vx, vy, vw = 0;

uint8_t flag = 0;
Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);

IMU_DATA imu_data;
ODOM_DATA odom_data;

TaskHandle_t control_task;
SemaphoreHandle_t control_sem;
SemaphoreHandle_t update_sem;
hw_timer_t *htim0;

void setup() {
  Serial.begin(115200);
  Serial.print("Heloo wooo");

  neopixelWrite(21, 0, 10, 0);
  Mobile.begin();
  Serial.print("Heloo wooo");

  neopixelWrite(21, 0, 0, 10);
  // delay(5000);
  control_sem = xSemaphoreCreateBinary();
  update_sem = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(control_loop,
                          "control-task-loop",
                          8192,
                          NULL,
                          tskIDLE_PRIORITY,
                          &control_task,
                          0);

  htim0 = timerBegin(1000000);
  timerAttachInterrupt(htim0, &timer0_callback);
  timerAlarm(htim0, 1000, true, 0);
  neopixelWrite(21, 10, 0, 0);

  timestamp_cmd = micros();
}

void timer0_callback() {
  BaseType_t task_woken;
  xSemaphoreGiveFromISR(control_sem, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}

void control_loop(void *pv) {
  while (true) {
    static int counter = 0;
    if (xSemaphoreTake(control_sem, 0) == pdTRUE) {
      Mobile.control(vx, vy, vw);
      counter = (counter + 1) % 10;
      if (!counter) xSemaphoreGive(update_sem);
    }
  }
}

void loop() {
  uint64_t time = micros();
  if (xSemaphoreTake(update_sem, 0) == pdTRUE) {

    //imu_data = Mobile.getIMU();
    odom_data = Mobile.getODOM();


    // Serial.print(Mobile.qd_target[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[2]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[3]);
    // Serial.print(" ");
    
    // Serial.print(Mobile.fb_qd[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[2]);
    // Serial.print(" ");
    // Serial.print(Mobile.fb_qd[3]);
    // Serial.println(" ");
 
    Serial.print(encx[0]->get_diff_count());
    Serial.print(" ");
     Serial.print(encx[1]->get_diff_count());
    Serial.print(" ");
     Serial.print(encx[2]->get_diff_count());
    Serial.print(" ");
    Serial.println(encx[3]->get_diff_count());

    // Serial.print(15);
    // Serial.print(" ");
    // Serial.println(-15);
  }

  //Cmd loop
  if (time - timestamp_cmd > timestep_cmd) {
    timestamp_cmd = time;
    if (flag == 0) {
      flag = 1;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 1) {
      flag = 2;
      vx = 0.25;
      vy = 0;
      vw = 0;
    } else if (flag == 2) {
      flag = 3;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 3) {
      flag = 4;
      vx = 0;
      vy = 0.25;
      vw = 0;
    } else if (flag == 4) {
      flag = 5;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 5) {
      flag = 6;
      vx = 0;
      vy = 0;
      vw = 0.25;
    } else if (flag == 6) {
      flag = 7;
      vx = 0;
      vy = 0;
      vw = 0;
    }
    else if (flag == 7) {
      flag = 8;
      vx = 0;
      vy = 0.25;
      vw = 0.25;
    }
    else if (flag == 8) {
      flag = 8;
      vx = 0;
      vy = 0;
      vw = 0;
    }
  }
}
