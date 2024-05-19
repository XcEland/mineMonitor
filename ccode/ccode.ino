#include <Arduino.h>
#include <WiFi.h>               //we are using the ESP32
#include <Firebase_ESP_Client.h>
#include <DHT.h>         // Install DHT library by adafruit 1.3.8
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>  // Include math library for log function
//
// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeAcc = 0;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
float accX, accY, accZ;
float temperature;

// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

//
#define DHT_SENSOR_PIN 15
#define DHT_SENSOR_TYPE DHT22
//To provide the ESP32 / ESP8266 with the connection and the sensor type
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

const int Buzzer = 14; // Pin for Buzzer
const int smokeA0 = 34; //pin for smoke detector
int sensorThres = 100;
int data;
const float BETA = 3950; // Beta Coefficient of the thermistor
int sensitivity = 200; // Adjust this value based on your calibration

//
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Chimuka 03"
#define WIFI_PASSWORD "Passw0rd"


// Insert Firebase project API Key
#define API_KEY "AIzaSyBDPYx1rwKXrhKWa0V7JXWd_I2gJAg9AiM"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://minermonitor-34c00-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;                     //since we are doing an anonymous sign in
//
void setup(){
  dht_sensor.begin();
  Serial.begin(9600);
  delay(1000);
  //
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  initMPU();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok FIREBASE");
    Serial.print("Connected with IP FIREBASE: ");
    Serial.println(WiFi.localIP());
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  //

  analogReadResolution(10); // Set the ADC resolution to 10 bits
  // pinMode(15, INPUT); // Thermistor is connected to GPIO 15
  pinMode(Buzzer, OUTPUT); // Set Buzzer pin as output
  pinMode(smokeA0, INPUT);
}

void loop(){
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  Serial.println("Acc on x axes: ");
  Serial.println(accX);
  Serial.println("Acc on y axes: ");
  Serial.println(accY);
  Serial.println("Acc on z axes: ");
  Serial.println(accZ);
  
  float temperature = dht_sensor.readTemperature();
  float humidity = dht_sensor.readHumidity();
  digitalWrite(Buzzer, HIGH);

  data = analogRead(smokeA0);
  int air_quality = data * sensitivity / 1023;
  // Serial.println("Air quality: ");
  // Serial.println(air_quality);

  // Serial.print("Temperature : ");
  // Serial.println(temperature);

  // Serial.print("Humidity : ");
  // Serial.println(humidity); 

  // int analogValue = analogRead(15); // Read analog value from pin 15
  // Serial.print(analogValue);
  // float celsius = 1 / (log(1 / (1023.0 / analogValue - 1)) / BETA + 1.0 / 298.15) - 288.15;
  // Serial.print("Temperature: ");
  // Serial.print(celsius);
  // Serial.println(" ℃");

  // if(celsius >= 16){
  //   digitalWrite(Buzzer, HIGH); // Turn buzzer on
  // } else {
  //   digitalWrite(Buzzer, LOW); // Turn buzzer off
  // }
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)){
    //since we want the data to be updated every second
    sendDataPrevMillis = millis();
    // Enter Temperature in to the DHT_22 Table
    if (Firebase.RTDB.setInt(&fbdo, "DHT_22/Temperature", temperature)){
      // This command will be executed even if you dont serial print but we will make sure its working
      Serial.print("Temperature : ");
      Serial.println(temperature);
    }
    else {
      Serial.println("Failed to Read from the Sensor");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    // Enter Humidity in to the DHT_22 Table
    if (Firebase.RTDB.setFloat(&fbdo, "DHT_22/Humidity", humidity)){
      Serial.print("Humidity : ");
      Serial.print(humidity);
    }
    else {
      Serial.println("Failed to Read from the Sensor");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    // Enter Air Quality in to the MQ_Sensor Table
    if (Firebase.RTDB.setInt(&fbdo, "MQ_Sensor/AirQuality", air_quality)){
      // This command will be executed even if you dont serial print but we will make sure its working
      Serial.print("Air quality: ");
      Serial.print(air_quality);
    }
    else {
      Serial.println("Failed to Read from the Sensor");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
  delay(1000); // Delay for 1 second before the next reading

}
