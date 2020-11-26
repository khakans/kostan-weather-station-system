#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

int valTemp, valHumi, valPres, valAlti, valLumi, valAirq;
int ledPin = 27;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//CONNECTION
const char* ssid = "YOUR WIFI SSID";
const char* password = "YOUR WIFI PASSWORD";

//define mqtt
const char* mqttServerIP = "broker.hivemq.com";
const int mqttPort = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

//define topic
char* OneTopic = "kws/sensorKws";

void reconnect(){
  // MQTT Begin
  while(!client.connected()){
    Serial.println("Connecting to MQTT Server..");
    Serial.print("IP MQTT Server : "); Serial.println(mqttServerIP);
    bool hasConnection = client.connect("kans");
    if(hasConnection){
      Serial.println("Success connected to MQTT Broker");
    } else {
      Serial.print("Failed connected");
      Serial.println(client.state());
      digitalWrite(4,HIGH);
      delay(2000);
      Serial.println("Try to connect...");
    }
  }
  client.publish(OneTopic, "Reconnecting");
}

void callback(char* topic, byte* payload, unsigned int length){
  Serial.println("--------");
  Serial.println("Message Arrived");
  Serial.print("Topic :"); Serial.println(topic);
  Serial.print("Message : ");
  String pesan = "";
  for(int i=0; i < length; i++){
    Serial.print((char)payload[i]);
    pesan += (char)payload[i];
  }
  if(pesan == "ON" ){
    Serial.println("LED ON.. Warning");
    digitalWrite(4,HIGH);
  } else if(pesan == "OFF"){
    Serial.println("LED OFF.. Safety");
    digitalWrite(4,LOW);
  }
  Serial.println("ESP/CMD topic arrived");
  Serial.println(pesan);
  Serial.print("Pesan masuk :");
  Serial.println(pesan);
  Serial.println("--------");
}

//BME280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

//TSL2561
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void configureSensor(void)
{
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);      /* fast but low resolution */
}
 
void setup() {
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  pinMode(4,OUTPUT);
  
  Serial.begin(9600);
  if (!bme.begin(0x76)) {
    Serial.println("BME280 no detected");
    while (1);
  }
  if(!tsl.begin())
  {
    Serial.print("TSL2561 no detected");
    while(1);
  }
  displaySensorDetails();
  configureSensor();
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServerIP, mqttPort);
  client.setCallback(callback);
  delay(20);
}

void Connection(){
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop()){
    client.connect("Client");
  }
}

char dataPublish[50];
void publishMQTT(char* topics, String data){
   data.toCharArray(dataPublish, data.length() + 1);
   client.publish(topics, dataPublish);
}

int ledState = 0,ledStateN = HIGH;
unsigned long previousMillis = 0,previousMillisStream = 0;
const long interval = 1000,intervalStream = 500; 

void ledBlink(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == 0) {
      ledState = 16 ;
    } else {
      ledState = 0;
    }
    ledcWrite(ledChannel, ledState);
  }
}

void dataStream(){
  unsigned long currentMillisStream = millis();
  if (currentMillisStream - previousMillisStream >= intervalStream) {
    previousMillisStream = currentMillisStream;
    publishMQTT(OneTopic,(String(valTemp) + "," + String(valHumi) + "," + String(valLumi) + "," + String(valAirq) + "," + String(valPres) + "," + String(valAlti)));
    Serial.println("Temp: " + String(valTemp) + " *C | Humi: " + String(valHumi) + " % | Lumi: " + String(valLumi) + " Lux | AirQ: " + String(valAirq) + " ppm | Pres: " + String(valPres) + " hPa | Alti: " + String(valAlti) + " m" );
  }
}

void loop() {
  Connection();
   
  sensors_event_t event;
  tsl.getEvent(&event);
  
  valTemp = bme.readTemperature();
  valHumi = bme.readHumidity();
  valLumi = event.light;
  valAirq = (analogRead(12),DEC);
  valPres = bme.readPressure() / 100.0F;
  valAlti = bme.readAltitude(SEALEVELPRESSURE_HPA);

  ledBlink();
  
  dataStream();
  
  delay(50);
}
