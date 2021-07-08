#include <Arduino.h>
#include <dht.h>
#include <Adafruit_Sensor.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #define DHT_apin 13
#endif
#ifdef ESP32
  #include <WiFi.h>
  #define DHT_apin 36
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define TEMP_OFFSET 1.4
#define MQTT_CLIENT_ID  "Humidity_Sensor"
#define MQTT_SENSOR_TOPIC "humidity/sensor1"
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  300

DHT dht(DHT_apin, DHTTYPE);

const char* ssid = "";
const char* password = "";


const char broker[] = "";
uint16_t port = 1883;


long lastMsg = 0;

float temperature;
float humidity;


WiFiClient espClient;
PubSubClient client(espClient);

//function declarations
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();
void print_wakeup_reason();

void setup() {

  Serial.begin(9600);
  while(!Serial){
    // wait for serial port to connect
  }
#ifdef esp32
  print_wakeup_reason();
#endif
  //Initialiaze sensor
  dht.begin();
  delay(500);//Delay to let system boot
  Serial.println("DHT22 Humidity & temperature Sensor\n\n");
  delay(1000);//Wait before accessing Sensor

  //Initialize WIFI and wait till connected
  
  setup_wifi();
  client.setServer(broker,port);
  //client.setCallback(callback);

}

void loop() {
  if (!client.connected()){
    reconnect();
  }
  client.loop();

    temperature = dht.readTemperature()-TEMP_OFFSET;
    char tempString[8];
    dtostrf(temperature,1,2,tempString);
    Serial.print("temperature = ");
    Serial.print(tempString); 
    Serial.println("C  ");

    humidity = dht.readHumidity();
    char humString[8];
    dtostrf(humidity,1,2,humString);
    Serial.print("Current humidity = ");
    Serial.print(humString);
    Serial.print("%  ");

    StaticJsonDocument<300> doc;
    doc["device"] = "Humi_Sensor";
    doc["temperature"] = tempString;
    doc["humidity"] = humString;

    char buffer[256];
    serializeJsonPretty(doc, Serial);
    Serial.println();

    serializeJson(doc, buffer);

    client.publish(MQTT_SENSOR_TOPIC, buffer);

    #ifdef esp32
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP*uS_TO_S_FACTOR);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    #endif

    Serial.println("Sleep ...");
    Serial.flush();
    #ifdef ESP32
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP*uS_TO_S_FACTOR);
    #endif
    #ifdef ESP8266
      ESP.deepSleep(TIME_TO_SLEEP*uS_TO_S_FACTOR);
    #endif

  //}


}

void setup_wifi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.println("Connecting to WiFi..");
}
  Serial.println("You're connected to the network");
  Serial.println();
}

void callback(char* topic, byte* message, unsigned int length){
//can be added to read incoming messages
}

void reconnect(){
    while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      // client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#ifdef esp32
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
#endif


