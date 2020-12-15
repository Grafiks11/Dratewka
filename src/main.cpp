#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Seeed_BME280.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>


SoftwareSerial pmsSerial(18, 19);
BME280 bme280;

//SSID Password combination
const char* ssid = "fill-in";
const char* password = "fill-in";

//MQTT Broker IP address:
const char* mqtt_server = "fill-in for example: 192.168.137.46";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

int gas_din=14; //alcohol, benzyn, amoniak
int gas_ain=34;
int ad_value;
int co_din=25;//CO2
int co_ain=26;
int ad_value2;
 
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();
float temperature=0;
float pressure=0;
float altitude=0;
float humidity=0;

void setup() {

  pinMode(co_din,INPUT);
  pinMode(co_ain,INPUT);
  pinMode(gas_din,INPUT);
  pinMode(gas_ain,INPUT);
  pinMode(15,OUTPUT); //3.3V supply
  pinMode(4,OUTPUT);  //5V supply
  // Debugging output
  Serial.begin(115200);
  
  if (!bme280.init()) {
        Serial.println("Device error!");
    }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pmsSerial.begin(9600);

}

void setup_wifi() {
  delay(10);
  // Start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      
    }
    else if(messageTemp == "off"){
      Serial.println("off");

    }
  }
}

void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a struct 
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}


void loop() {
float pressure;

digitalWrite(4,LOW);
digitalWrite(15,LOW);

if (!client.connected()) {
    reconnect();
  }
  client.loop();

long now = millis();
  if (now - lastMsg > 5000) { //publish every 5 seconds
    lastMsg = now;
 temperature = bme280.getTemperature();   
 pressure=bme280.getPressure(); 
 altitude=bme280.calcAltitude(pressure); 
 humidity=bme280.getHumidity(); 

  
   char tempString[8];
    dtostrf(temperature, 1, 2, tempString);  //convert to a char array
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

   char preString[8];
    dtostrf(pressure, 1, 2, preString);  
    Serial.print("Pressure: ");
    Serial.println(preString);
    client.publish("esp32/pressure", preString);

   char altString[8];
    dtostrf(altitude, 1, 2, altString);  
    Serial.print("Altitude: ");
    Serial.println(altString);
    client.publish("esp32/altitude", altString);

   char humString[8];
    dtostrf(humidity, 1, 2, humString);  
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);

    ad_value=analogRead(gas_ain);
   char adString[8];
    dtostrf(ad_value, 1, 2, adString);
    Serial.print("Alcohol: ");
    Serial.println(adString);
    client.publish("esp32/alcohol", adString);

    ad_value2=analogRead(co_ain);
   char ad2String[8];
    dtostrf(ad_value2, 1, 2, ad2String);
    Serial.print("CO2: ");
    Serial.println(ad2String);
    client.publish("esp32/co2", ad2String);

readPMSdata(&pmsSerial);

  char pm10String[8];
    dtostrf(data.pm10_standard, 1, 2, pm10String);
    Serial.print("PM 1.0: ");
    Serial.println(pm10String);
    client.publish("esp32/PM1.0", pm10String);

  char pm25String[8];
    dtostrf(data.pm25_standard, 1, 2, pm25String);
    Serial.print("PM 2.5: ");
    Serial.println(pm25String);
    client.publish("esp32/PM2.5", pm25String);

  char pm100String[8];
    dtostrf(data.pm100_standard, 1, 2, pm100String);
    Serial.print("PM 10.0: ");
    Serial.println(pm100String);
    client.publish("esp32/PM10", pm100String);
  
  }
}

