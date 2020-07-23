/*
 * Error              Meaning             Action
 * LED flashes once   BME680 not found    Make sure the BME680 is wired in and working. If using a non-standard I2C mode, you may have to update the BME680_I2C_MODE configuration.
 * LED flashes twice  WiFi error          Make sure wifi can reach the ESP32, and the NETWORK_ID/PASSWORD configuration is correct.
 * LED flashes trebly MQTT client error   Make sure your MQTT broker is running and the connection settings are correct.
 * Solid Light        No error!           Have some tea. Relax. Read a book.
 */


/* ---------- START of configuration area ---------- */
/* CONFIG WiFi */
#define NETWORK_ID "YOUR WIFI GOES HERE"
#define PASSWORD "YOUR WIFI PASSWORD GOES HERE"

/* CONFIG BME680 */
#define BME680_I2C_MODE I2C_STANDARD_MODE

/* CONFIG MQTT */
#define MQTT_TOPIC "house_env"
/* format mqtt://username:password@host:port */
/* format mqtts://username:password@host:port for ssl*/
/* see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html for more info */
#define MQTT_URI "mqtt://192.168.1.1:1883"
/* OPTIONAL */ 
#define MQTT_LWT_QOS 2
#define MQTT_LWT_RETAIN 0

/* CONFIG general */
#define DEBUG 1
#define BME680_ATTEMPTS 5U
#define CONVERT_TO_FAHRENHEIT //Comment this out for Celsius
#define INTERVAL 60000 //Milliseconds between measurements/publishings.
#define SERIAL_SPEED 115200
/* ---------- END of configuration area ---------- */


#include "Zanshin_BME680.h"
#include <WiFi.h>
#include "mqtt_client.h"

#define STR(X) #X
#ifdef DEBUG
#define PRINT(X) Serial.print(F(X "\n"))
#define PRINTF(X, ...) Serial.printf(X "\n", __VA_ARGS__)
#else
#define PRINT(X) {}
#define PRINTF(X, ...) {}
#endif


BME680_Class BME680;
esp_mqtt_client_config_t mqtt_config = {};
static uint8_t soft_errors = 0;

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
setup_BME680(void)
{
  uint8_t attempts = BME680_ATTEMPTS;

  PRINT("- Initializing BME680 sensor...");
  for (;;) {
    if(BME680.begin(BME680_I2C_MODE)) {
      BME680.setOversampling(TemperatureSensor, Oversample16);
      BME680.setOversampling(HumiditySensor, Oversample16);
      BME680.setOversampling(PressureSensor, Oversample16);
      BME680.setIIRFilter(IIR4);
      BME680.setGas(320,150);
      return;
    }

    if (--attempts == 0) {
      PRINT("CRITICAL - couldn't find BME680 after " STR(BME680_ATTEMPTS) "attempts.");
      error_loop(1);
      __builtin_unreachable();
    }

    PRINT("WARNING - couldn't find BME680. Retrying in 2 seconds.");
    delay(2000);
  }
  __builtin_unreachable();
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

esp_mqtt_client_handle_t
setup_mqtt()
{
  esp_mqtt_client_handle_t out;
  char buf[32];
  
  PRINT("- Initializing MQTT Client");
  mqtt_config.uri = MQTT_URI;
  mqtt_config.lwt_qos = MQTT_LWT_QOS;
  mqtt_config.lwt_msg = "disconnected";
  mqtt_config.lwt_topic = MQTT_TOPIC "/lwt";
  out = esp_mqtt_client_init(&mqtt_config);
  if (out == NULL) {
    PRINT("CRITICAL - couldn't create MQTT client.");
    goto error;
  }
  if (esp_mqtt_client_start(out) != ESP_OK) {
    PRINT("CRITICAL - couldn't start MQTT client.");
    goto error;
  }

  delay(1000);
  snprintf(buf, sizeof(buf), "connected");
  esp_mqtt_client_publish(out, mqtt_config.lwt_topic, buf, strlen(buf), 1, 0);
  return out;
  
error:
  error_loop(3);
  __builtin_unreachable();
}

static esp_mqtt_client_handle_t mqttc;
void
setup() {
  Serial.begin(SERIAL_SPEED);
  pinMode(LED_BUILTIN, OUTPUT);
  PRINT("Starting ESP32 Thing I2C-MQTT Environmental Sensor.");
  setup_BME680();
  setup_wifi();
  mqttc = setup_mqtt();
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
  PRINTF("Publishing to MQTT broker... soft errors %d", soft_errors);

  /* Temperature */
#ifdef CONVERT_TO_FAHRENHEIT
  temp = ((temp * 9) / 5) + 3200;
#endif
  snprintf(buf, sizeof(buf), "%d.%02d", temp / 100, temp % 100);
  soft_errors += esp_mqtt_client_publish(mqttc, MQTT_TOPIC "/temp", buf, strlen(buf), 1, 0) == 0;

  /* humidity */
  snprintf(buf, sizeof(buf), "%d.%02d", humidity / 1000, (humidity % 1000)/10);
  soft_errors += esp_mqtt_client_publish(mqttc, MQTT_TOPIC "/humidity", buf, strlen(buf), 1, 0) == 0;

  /* pressure */
  snprintf(buf, sizeof(buf), "%d", pressure);
  soft_errors += esp_mqtt_client_publish(mqttc, MQTT_TOPIC "/pressure", buf, strlen(buf), 1, 0) == 0;

  /* gas */
  snprintf(buf, sizeof(buf), "%d", gas);
  soft_errors += esp_mqtt_client_publish(mqttc, MQTT_TOPIC "/gas", buf, strlen(buf), 1, 0) == 0;

  PRINTF("Finished publishing... soft errors %d", soft_errors);
}

void
loop()
{
  if (soft_errors >= 10) {
    PRINT("WARNING - encountered 10 non-fatal errors since reset, resetting");
    ESP.restart();
  }
  PRINT("ESP32 Thing I2C-MQTT Environmental Sensor Main Loop.");
  BME680.getSensorData(temp,humidity,pressure,gas,true);
  PRINTF("Temp: %d celsius, humidity: %d percent, pressure: %d Pascals, gas: %d ohms", temp, humidity / 1000, pressure, gas);
  mqtt_publish();
  delay(INTERVAL);
}
