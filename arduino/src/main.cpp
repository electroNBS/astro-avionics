#include <Arduino.h>

// define pins
#define BuzzerPin A1

// define Constants
#define BOOT_TIME 15000    // time to wait for both systems to boot up
#define CONNECT_TIME 15000 // time to wait for the connection to be established

// define Gobal variables
unsigned long startTime = 0;
int connectionStatus = 0; // 0 if fail , 1 if success
float currentAltitude = 0;
float baseAltitude = 0;
float bmpVelocity = 0; // velocity of the rocket calculated by bmp altitude readings
int bmpWorking = 0; // 0 if fail , 1 if success

// define task handle
TaskHandle_t buzzerTask;

// define buzzer task
void buzzerTask(void *pvParameters)
{
  // initialize the buzzer
  pinMode(BuzzerPin, OUTPUT);

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

float getAltitude(){
  // get the altitude from the bmp sensor
  // TODO: implement this function
  return 0;
}
void bmpTask(void *pvParameters)
{
  // initialize the sensor
  // TODO : set bmpworking 1 if intialized successfully , else 0

  // check if the sensor is working
  if (bmpWorking == 0)
  {
    // if the sensor is not working, set the base altitude to 0
    baseAltitude = 0;
    // delete task
    vTaskDelete(NULL);
  }
  else
  {
    // if the sensor is working, set the base altitude to the current altitude
    baseAltitude = getAltitude();
    // TODO : can take average of 10 readings every second to get the base altitude more accurately
  }
  while (millis() < BOOT_TIME)
  {
    // wait for the boot time to pass
    ;
  }
  
  unsigned long t1 = millis();
  unsigned long t2 = millis();
  // set the start time
  while (1)
  {
    // get the current time 
    t2 = millis();
    // calculate the velocity of the rocket
    float reading = getAltitude();
    bmpVelocity = (reading - baseAltitude) / (t2 - t1);
    // get the current altitude every 10 ms
    currentAltitude = reading;

    // update the time
    t1 = t2;
    
    // wait for 10ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
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


  // create tasks
  /*
   * @param "buzzerTask" The name of the task.
   * @param 1024 The stack size allocated for the task.
   * @param NULL The parameter passed to the task (none in this case).
   * @param 1 The priority of the task.
   * @param NULL The task handle (not used in this case).
   */
  xTaskCreate(buzzerTask, "buzzerTask", 1024, NULL, 1, NULL); // create buzzer task
  xTaskCreate(bmpTask, "bmpTask", 2048, NULL, 1, NULL);       // create bmp task 
  // NOTE: increase the stack size if required
  // NOTE: increase the priority if required

}

void loop()
{
}