#include <Arduino.h>
#include <dht.h>
#include <Adafruit_Sensor.h>
#include <secrets.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #define DHT_apin 13
  #define DHT_power 12
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
#define SENSITIVITY (3.3 / 1024.0)

DHT dht(DHT_apin, DHTTYPE);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PW;


const char broker[] = BROKER;
uint16_t port = 1883;


long lastMsg = 0;

float temperature;
float humidity;
int batteryPct;


WiFiClient espClient;
PubSubClient client(espClient);

//function declarations
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();
void print_wakeup_reason();
int getBatteryPercentage();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {

  Serial.begin(9600);
  while(!Serial){
    // wait for serial port to connect
  }
#ifdef esp32
  print_wakeup_reason();
#endif
  //Initialiaze sensor


  //Initialize WIFI and wait till connected
  delay(500);//Delay to let system boot
  Serial.println("DHT22 Humidity & temperature Sensor\n\n");
  pinMode(DHT_power, OUTPUT);
  digitalWrite(DHT_power,LOW);

  //client.setCallback(callback);

}

void loop() {
    digitalWrite(DHT_power, HIGH); //power the sensor
    delay(500);//wait for pin to become high and power the sensor
    dht.begin();
    delay(1000);//Wait before accessing Sensor
    temperature = dht.readTemperature()-TEMP_OFFSET;
    humidity = dht.readHumidity();
    digitalWrite(DHT_power, LOW); //power down sensor to save battery

    char tempString[8];
    dtostrf(temperature,1,2,tempString);

    char humString[8];
    dtostrf(humidity,1,2,humString);

    float batteryRaw = analogRead(A0);

    Serial.print("temperature = ");
    Serial.print(tempString); 
    Serial.println("C  ");
    Serial.print("Current humidity = ");
    Serial.print(humString);
    Serial.print("%  ");

    batteryPct = getBatteryPercentage();

    StaticJsonDocument<300> doc;
    doc["device"] = "Humi_Sensor";
    doc["temperature"] = tempString;
    doc["humidity"] = humString;
    doc["BatteryPct"] = batteryPct;
    doc["BatteryRaw"] = batteryRaw;

    char buffer[256];
    serializeJsonPretty(doc, Serial);
    Serial.println();

    serializeJson(doc, buffer);

    setup_wifi(); //start wifi
    client.setServer(broker,port); //set mqtt broker
    if (!client.connected()){
      reconnect();
    }
    client.loop();

    client.publish(MQTT_SENSOR_TOPIC, buffer); //send data to mqtt
    delay(1000); //delay to give some time to actually send the data (without delay, data is not sent)
    WiFi.disconnect();

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

int getBatteryPercentage(){
    float batteryValue = analogRead(A0);
    Serial.print("Raw battery value: ");
    Serial.println(int(batteryValue));

    batteryValue = batteryValue * SENSITIVITY;
    Serial.print("Voltage: ");
    Serial.print(batteryValue);
    Serial.println("V");

    batteryValue = mapf(batteryValue, 1.98, 2.65, 0, 100);
    Serial.print("Battery percentage: ");
    Serial.print(int(batteryValue));
    Serial.println("%");

    return batteryValue;

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  float a = x - in_min;
  float b = out_max - out_min;
  float c = in_max - in_min;
  return a * b / c + out_min;
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


