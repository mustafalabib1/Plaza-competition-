#include <Arduino.h>

int count1 = 0;
int count2 = 0;
TaskHandle_t task1_handle = NULL;

void task1(void* parameters) {
  for (;;) {
    Serial.print("Task 1 counter: ");
    Serial.println(count1++);
    Serial.print("task1 running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("--------------------------------------------------------------");
    Serial.println();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (count1 > 3) {
      vTaskSuspend(NULL);
    }
  }
}
void task2(void* parameters) {
  for (;;) {
    Serial.print("Task 2 counter: ");
    Serial.println(count2++);
    Serial.print("task2 running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("--------------------------------------------------------------");
    Serial.println();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  xTaskCreate(
    task1,    // function name
    "Task1",  // Task BaseType_t xTaskCreate(TaskFunction_t pvTaskCode,
    1000,     // const char *const pcName, const uint32_t usStackDepth,
    NULL,     // void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pvCreatedTask)
    1,
    NULL);
  xTaskCreate(
    task2,
    "Task2",
    1000,
    NULL,
    1,
    NULL);
}

void loop() {
  Serial.print("main loop() running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println("--------------------------------------------------------------");
  Serial.println();
  delay(2000);
  
  if (count1 > 3 && task1_handle != NULL) {
    vTaskSuspend(task1_handle);
  }

  if (count2 == 5 && task1_handle != NULL) {
    vTaskResume(task1_handle);
  }
}