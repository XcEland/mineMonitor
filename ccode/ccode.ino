#include <Arduino.h>
#include <WiFi.h>               //we are using the ESP32
#include <Firebase_ESP_Client.h>
#include <DHT.h>         // Install DHT library by adafruit 1.3.8
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>  // Include math library for log function
//
#define LED_RED 5       // Pin where the LED is connected
#define LED_BLUE 4       // Pin where the LED is connected
// Timer variables
unsigned long lastTime = 0;
float deltaTime;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float displacementX = 0, displacementY = 0, displacementZ = 0;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
float accX, accY, accZ;
float temperature;

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
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

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
  pinMode(Buzzer, OUTPUT); // Set Buzzer pin as output
  pinMode(smokeA0, INPUT);
  lastTime = millis();
}

void loop(){
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // deltaTime in seconds
  lastTime = currentTime;

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  accX += 104.2;
  accY += 77.2;

  Serial.print("Acc on x axes: ");
  Serial.print(accX);
  Serial.print("Acc on y axes: ");
  Serial.print(accY);
  Serial.print("Acc on z axes: ");
  Serial.print(accZ);
  Serial.println(" m/s^2");

  // Convert raw accelerometer data to acceleration in m/s^2
  // Assuming a scale factor for the accelerometer: 16384 LSB/g (for +/- 2g range)
  float ax = accX / 16384.0 * 9.81; // Convert to m/s^2
  float ay = accY / 16384.0 * 9.81; // Convert to m/s^2
  float az = accZ / 16384.0 * 9.81; // Convert to m/s^2

  // Integrate acceleration to get velocity
  velocityX += ax * deltaTime;
  velocityY += ay * deltaTime;
  velocityZ += az * deltaTime;

  // Integrate velocity to get displacement
  displacementX = velocityX * deltaTime;
  displacementY = velocityY * deltaTime;
  displacementZ = velocityZ * deltaTime;

  // Round displacement to 1 decimal place
  displacementX = round(displacementX * 10) / 10.0;
  displacementY = round(displacementY * 10) / 10.0;
  displacementZ = round(displacementZ * 10) / 10.0;
  
  Serial.println("displacement X: ");
  Serial.print(displacementX);
  Serial.print(", Y: ");
  Serial.print(displacementY);
  Serial.print(", Z: ");
  Serial.print(displacementZ);
  Serial.println(" m/s^2");
  //

  float temperature = dht_sensor.readTemperature();
  float humidity = dht_sensor.readHumidity();
  digitalWrite(Buzzer, HIGH);

  data = analogRead(smokeA0);
  int air_quality = data * sensitivity / 1023;
  //
  if (temperature > 20 && air_quality < 100 && temperature < 30) {
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(Buzzer, HIGH);
  } else if (temperature < 20 || air_quality > 100) {
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(Buzzer, HIGH);
  } else {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(Buzzer, LOW);
  }
  //
  
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

    // Enter Air Quality in to the Displacement Table
    if (Firebase.RTDB.setInt(&fbdo, "Displacement/displacementX", displacementX)){
      // This command will be executed even if you dont serial print but we will make sure its working
      Serial.print("displacementX: ");
      Serial.print(displacementX);
    }
    if (Firebase.RTDB.setInt(&fbdo, "Displacement/displacementY", displacementY)){
      // This command will be executed even if you dont serial print but we will make sure its working
      Serial.print("displacementY: ");
      Serial.print(displacementY);
    }
    if (Firebase.RTDB.setInt(&fbdo, "Displacement/displacementZ", displacementZ)){
      // This command will be executed even if you dont serial print but we will make sure its working
      Serial.print("displacementZ: ");
      Serial.print(displacementZ);
    }
  }
  delay(1000); // Delay for 1 second before the next reading

}
