#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <driver/dac.h>
#include <Wire.h>
#include <SI114X.h>
#include <Adafruit_INA219.h>
#include <Stepper.h>

#define STATIC 1
#define TRACKING 2

#define TEMPERATURE_PIN 34

#define IN4A 32
#define IN3A 33
#define IN2A 25
#define IN1A 26

#define IN4B 19
#define IN3B 18
#define IN2B 17
#define IN1B 16

#define STEPS 200

const boolean testMode = true;

const int B = 4255;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k

const int reqCapacity = JSON_OBJECT_SIZE(6);


//Stepper stepperA(STEPS, IN1A, IN2A, IN3A, IN4A);
Stepper stepper(STEPS, IN1B, IN2B, IN3B, IN4B);

SI114X SI1145 = SI114X();
Adafruit_INA219 ina219;

WiFiServer server(80);

const char* ssid     = "Group25";
const char* password = "supersecretpw124";

void initSunlightSensor() {
  Serial.println("Intializing Si1145 (Sunlight)...");
  while (!SI1145.Begin()) {
      Serial.println("Si1145 is not ready!");
      delay(1000);
  }
  Serial.println("Si1145 is ready!");
}

void initDCSensor() {
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void initTemperatureSensor() {
  Serial.println("Intializing INA219 (DC)...");
  pinMode(TEMPERATURE_PIN, INPUT);
  Serial.println("Temperature sensor is ready!");
}

float getTemperature() {
  int a = analogRead(TEMPERATURE_PIN);
  float R = 4095.0/a-1.0;
  R = R0*R;
  float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15; // convert to temperature via datasheet
  return temperature;
}

float getLumens() {
  return (float) SI1145.ReadVisible();
}

float getIR() {
  return (float) SI1145.ReadIR();
}

float getUV() {
  return (float) SI1145.ReadUV();
}

float getCurrent() {
  float current_mA = 0;

  current_mA = ina219.getCurrent_mA();

  return current_mA;
}

float getVoltage() {
  float shuntvoltage = 0;
  float busvoltage = 0;
  float loadvoltage = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  return loadvoltage;
}

void initWiFi() {
  // Station mode?
  WiFi.mode(WIFI_STA); //Optional
  
  // Connect to WiFi network
  Serial.print("\nAttempting to connect to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Progress prints
  int count = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if ((count+1) % 10 == 0) {
      Serial.println();
    }
    Serial.print(".");
    delay(500);
    count = count + 1;
  }
  
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

int last;

void setup() {
  // Connect to serial monitor
  Serial.begin(115200);
//  pinMode(5, OUTPUT);      // set the LED pin mode
  delay(100);
  Serial.println("Running setup:");
  Serial.println("Intializing sensors...");
  initSunlightSensor();
  initDCSensor();
  initTemperatureSensor();
  Serial.println("Sensors ready!");
  
  initWiFi();
  
  stepper.setSpeed(3);
  
  if (testMode) {
    Serial.println("\nRunning in Test Mode");
    server.begin();
  }
  else {
    
  }

  last = millis();
}

void loop() {
  if (testMode) {
   WiFiClient client = server.available();   // listen for incoming clients
  
    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character
  
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
  
              // the content of the HTTP response follows the header:
              client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
              client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");
              
              String temp = "Temperature: ";
              temp.concat(getTemperature());
              temp.concat(" degrees Celsius<br>");
              client.print(temp);
              
//              String lum = "Lumens: ";
//              lum.concat(getLumens());
//              lum.concat(" lm<br>");
//              client.print(lum);
//              
//              String ir = "Lumens: ";
//              ir.concat(getIR());
//              ir.concat(" idk<br>");
//              client.print(lum);
//              
//              String uv = "UV index: ";
//              uv.concat(getUV());
//              uv.concat("<br>");
//              client.print(lum);
              
              String mAmps = "Current: ";
              mAmps.concat(getCurrent());
              mAmps.concat(" mA<br>");
              client.print(mAmps);
              
              String volts = "Voltage: ";
              volts.concat(getVoltage());
              volts.concat(" V<br>");
              client.print(volts);
              
              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
  
          // Check to see if the client request was "GET /H" or "GET /L":
          if (currentLine.endsWith("GET /H")) {
            digitalWrite(5, HIGH);               // GET /H turns the LED on
          }
          if (currentLine.endsWith("GET /L")) {
            digitalWrite(5, LOW);                // GET /L turns the LED off
          }
        }
      }
      // close the connection:
      client.stop();
      Serial.println(millis());
      Serial.println("Client Disconnected.");
    }
  }
  else {
    if (WiFi.status() == WL_CONNECTED) {
        String server = "https://dynamic-website-327720.nn.r.appspot.com/writedata";
        float temperature = getTemperature();
//        float visVal = getLumens();
//        float irVal = getIR();
//        float uvVal = getUV();
        float visVal = 0;
        float irVal = 0;
        float uvVal = 0;
        float voltage1 = getVoltage();
        float current1 = getCurrent();
        float voltage2 = 3.3;
        float current2 = 150;

        updateServer(server, temperature, visVal, irVal, uvVal, voltage1, current1, voltage2, current2);
    }
    Serial.println("Sleeping for 1 minute...");

    int count = 0;
    while (count < 10) {
      stepper.step(-50);
      Serial.println("vertical");
      delay(2000);
      stepper.step(39);
      Serial.println("70 degrees");
      delay(2000);
      stepper.step(11);
      Serial.println("Horizontal");
      delay(2000);
      count = count + 1;
    }
    
  }
}

void updateServer(String server, float temperature, float visVal, float irVal, float uvVal, float voltage, float current, float volt2, float cur2) {
  // Create JSON document of sensor data + location
  String docid = "";
  int month = 4;
  int date = 4;
  int minutes = millis() / 60000 + 780; //780 = 1PM
  docid.concat(month);
  docid.concat(".");
  docid.concat(date);
  docid.concat(".");
  docid.concat(minutes);
  StaticJsonDocument<384> reqDoc;
  reqDoc["docID"] = docid;
  reqDoc["month"] = month;
  reqDoc["date"] = date;
  reqDoc["minutes"] = minutes;
  reqDoc["temperature"] = temperature;
  reqDoc["visible"] = visVal;
  reqDoc["ir"] = irVal;
  reqDoc["uv"] = uvVal;
  reqDoc["voltage1"] = voltage;
  reqDoc["current1"] = current;
  reqDoc["voltage2"] = volt2;
  reqDoc["current2"] = cur2;
  reqDoc["azimuth"] = 0;
  reqDoc["altitude"] = 0;
  reqDoc["debug"] = "None";

  // Convert it to String
  String output = "";
  serializeJson(reqDoc, output);
  Serial.print("Outgoing data packet:\t");
  Serial.println(output);
  Serial.println();

  // Get JSON of new position
  StaticJsonDocument<64> resDoc;
  DeserializationError err = deserializeJson(resDoc, POST_request(server, output));
  if (err) {
    // Deserializtion failed somehow
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.f_str());
  }
  else {
    Serial.println("no error");
  }
}


String POST_request(String server, String JSONpacket) {
  HTTPClient http;
  http.begin(server);
  // Specify content-type header
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(JSONpacket);

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();

  Serial.println(payload);
  return payload;
}
