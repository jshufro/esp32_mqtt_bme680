/*
 * Error              Meaning             Action
 * LED flashes twice  WiFi error          Make sure wifi can reach the ESP32, and the NETWORK_ID/PASSWORD configuration is correct.
 * LED flashes trebly MQTT client error   Make sure your MQTT broker is running and the connection settings are correct.
 * Solid Light        No error!           Have some tea. Relax. Read a book.
 */


/* ---------- START of configuration area ---------- */
/* CONFIG WiFi */
#define NETWORK_ID "YOUR WIFI GOES HERE"
#define PASSWORD "YOUR WIFI PASSWORD GOES HERE"

/* CONFIG MQTT */
#define MQTT_TOPIC "house_env"
/* format mqtt://username:password@host:port */
/* format mqtts://username:password@host:port for ssl*/
/* see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html for more info */
#define MQTT_URI "mqtt://192.168.1.1:1883"
/* OPTIONAL */ 
#define MQTT_LWT_QOS 2
#define MQTT_LWT_RETAIN 1
#define MQTT_RETAIN 0
#define MQTT_QOS 1

/* CONFIG Liquid Sensor */
#define LIQUID_SENSOR_GPIO GPIO_NUM_34

/* CONFIG general */
#define DEBUG 1
#define INTERVAL 60000 //Milliseconds between measurements/publishings.
#define SERIAL_SPEED 115200
#define WDT_TIMEOUT (INTERVAL / 1000) * 2 // Resets if we don't publish at least 1 datum every 2 intervals
/* ---------- END of configuration area ---------- */

#include <WiFi.h>
#include <esp_task_wdt.h>
#include "mqtt_client.h"

#define STR(X) #X
#ifdef DEBUG
#include "freertos/FreeRTOS.h"
#define PRINT(X) Serial.print(F(X "\n"))
#define PRINTF(X, ...) Serial.printf(X "\n", __VA_ARGS__)
#else
#define PRINT(X) {}
#define PRINTF(X, ...) {}
#endif


esp_mqtt_client_config_t mqtt_config = {};

void
flash_led(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(150);
  digitalWrite(LED_BUILTIN, LOW);
  delay(150);
  return;
}

void
error_loop(uint8_t blips)
{
  uint8_t restart_timer = 20;
  uint8_t blips_left;
  for(;restart_timer > 0; restart_timer--) {
    blips_left = blips;
    while(blips_left-- > 0) {
      flash_led();
    }
    delay(2000); /* 2 second pause inbetween error code repeats */
  }
  PRINT("CRITICAL - Flashed error code 20 times. Resetting device.");
  ESP.restart();
}

void
setup_wifi(void)
{
  int16_t timeout = 30000; /* milliseconds */
  
  PRINT("- Initializing WIFI connection with SSID " NETWORK_ID);
  WiFi.begin(NETWORK_ID, PASSWORD);
  delay(500);
  while (WiFi.status() != WL_CONNECTED) {
    if (timeout < 0) {
      PRINT("CRITICAL - couldn't connect to wifi after 30 seconds.");
      error_loop(2);
      __builtin_unreachable();
    }
    PRINT("...");
    delay(500);
    timeout = timeout - 500;
  }
}

int
mqtt_event_handler(esp_mqtt_event_t *event)
{
  char buf[32];
  static __thread bool wdt_added = false;

  if (event == NULL) {
    return 0;
  }

  switch(event->event_id) {
    default:
      return 0;
    case MQTT_EVENT_BEFORE_CONNECT:
      if (!wdt_added) {
        wdt_added = true;
        // Add current task to task watchdog timer
        PRINTF("Adding watchdog for task %s", pcTaskGetTaskName(NULL));
        esp_task_wdt_add(NULL);
      }
      break;
    case MQTT_EVENT_PUBLISHED:
      // Feed the watchdog every time we publish data. That means we're doing well.
      PRINTF("Feeding the watchdog for task %s", pcTaskGetTaskName(NULL));
      esp_task_wdt_reset();
      break;
    case MQTT_EVENT_CONNECTED:
      // Tell subscribers we're here
      snprintf(buf, sizeof(buf), "connected");
      esp_mqtt_client_publish(event->client, mqtt_config.lwt_topic, buf, strlen(buf), MQTT_LWT_QOS, MQTT_LWT_RETAIN);
  }
  return 0;
}

esp_mqtt_client_handle_t
setup_mqtt(void)
{
  esp_mqtt_client_handle_t out;
  
  PRINT("- Initializing MQTT Client");
  mqtt_config.uri = MQTT_URI;
  mqtt_config.lwt_qos = MQTT_LWT_QOS;
  mqtt_config.lwt_msg = "disconnected";
  mqtt_config.lwt_topic = MQTT_TOPIC "/lwt";
  mqtt_config.lwt_retain = MQTT_LWT_RETAIN;
  mqtt_config.event_handle = mqtt_event_handler;
  out = esp_mqtt_client_init(&mqtt_config);
  if (out == NULL) {
    PRINT("CRITICAL - couldn't create MQTT client.");
    goto error;
  }
  if (esp_mqtt_client_start(out) != ESP_OK) {
    PRINT("CRITICAL - couldn't start MQTT client.");
    goto error;
  }

  // Short delay on startup to allow mqtt background threads to connect
  delay(1000);
  return out;
  
error:
  error_loop(3);
  __builtin_unreachable();
}

void
setup_liquid_sensor(void)
{
  pinMode(LIQUID_SENSOR_GPIO, INPUT);
}

static esp_mqtt_client_handle_t mqttc;
void
setup() {
  Serial.begin(SERIAL_SPEED);
  pinMode(LED_BUILTIN, OUTPUT);
  setup_liquid_sensor();
  PRINT("Starting ESP32 Thing I2C-MQTT Liquid Sensor.");
  setup_wifi();
  mqttc = setup_mqtt();
  // Setup the watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  PRINTF("Adding watchdog for task %s", pcTaskGetTaskName(NULL));
  esp_task_wdt_add(NULL);
  PRINT("Setup done\n");

  /* Turn on the LED to indicate that the setup was successful. */
  digitalWrite(LED_BUILTIN, HIGH);
}

/*
 * temp - centidegrees Celsius - divmod by 100
 * humidity - milli-percent - divmod by 1000
 * pressure - Pascals - divmod by 1000 for kPa
 * gas - ohms - divmod by 1000 for kOhms
 */
static int32_t temp, humidity, pressure, gas;
void
mqtt_publish(void)
{
  char buf[32];
  PRINT("Publishing to MQTT broker.");

  /* wet/dry */
  snprintf(buf, sizeof(buf), "%s", digitalRead(LIQUID_SENSOR_GPIO) == 1 ? "dry" : "wet");
  esp_mqtt_client_publish(mqttc, MQTT_TOPIC "/liquid", buf, strlen(buf), MQTT_QOS, MQTT_RETAIN);

  PRINT("Finished publishing.");
}

void
loop()
{
  PRINT("ESP32 Thing I2C-MQTT Environmental Sensor Main Loop.");
  mqtt_publish();
  PRINTF("Feeding the watchdog for task %s", pcTaskGetTaskName(NULL));
  esp_task_wdt_reset();
  delay(INTERVAL);
}
