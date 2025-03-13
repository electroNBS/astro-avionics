#include <Arduino.h>

// define pins
#define BuzzerPin A1

// define Constants
#define BOOT_TIME 15000    // time to wait for both systems to boot up
#define CONNECT_TIME 15000 // time to wait for the connection to be established

// define Gobal variables
unsigned long startTime = 0;
int connectionStatus = 0; // 0 if fail , 1 if success

// define task handle
TaskHandle_t buzzerTask;

// define buzzer task
void buzzerTask(void *pvParameters)
{
  // we have to beep once every second till time is less than BOOT_TIME + CONNECT_TIME
  while (1)
  {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    if (connectionStatus == 1)
    {
      // beep for one second and then stop
      digitalWrite(BuzzerPin, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(BuzzerPin, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    if (elapsedTime < BOOT_TIME)
    {
      digitalWrite(BuzzerPin, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(BuzzerPin, LOW);
      vTaskDelay(900 / portTICK_PERIOD_MS);
    }
    else if (elapsedTime < BOOT_TIME + CONNECT_TIME)
    {
      digitalWrite(BuzzerPin, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(BuzzerPin, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (connectionStatus == 0)
    {

      // beap fast if connection unsuccessful
      digitalWrite(BuzzerPin, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(BuzzerPin, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelete(NULL);
    }
  }
}
/**
 * @brief Arduino setup function.
 *
 * Initializes serial communication at 115200 baud rate, sets pin modes,
 * and creates tasks.
 *
 * @param BuzzerPin The pin number connected to the buzzer.
 * @param buzzerTask The task function to handle buzzer operations.
 * @param "buzzerTask" The name of the task.
 * @param 1024 The stack size allocated for the task.
 * @param NULL The parameter passed to the task (none in this case).
 * @param 1 The priority of the task.
 * @param NULL The task handle (not used in this case).
 */
void setup()
{
  Serial.begin(115200);

  // set pin modes

  pinMode(BuzzerPin, OUTPUT);

  // create tasks
  /*
   * @param "buzzerTask" The name of the task.
   * @param 1024 The stack size allocated for the task.
   * @param NULL The parameter passed to the task (none in this case).
   * @param 1 The priority of the task.
   * @param NULL The task handle (not used in this case).
   */
  xTaskCreate(buzzerTask, "buzzerTask", 1024, NULL, 1, NULL); // create buzzer task
}

void loop()
{
}