#include <Wire.h>
#include <SPI.h>
#include "WiFiEsp.h"
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "SoftwareSerial.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// Emulate MySerial on pins 6/7 if not present for communicating with the network
#ifndef HAVE_HWSERIAL1

//Used in Esp8266 wifi modudle
SoftwareSerial Serial1(18, 19); // RX, TX
#endif

//Used to connect to network.
const char* ssid = "**********";            // your network SSID (name)
const char* pass = "**********";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
//Set the ip address or server name.
char server[] = "b44a-68-194-23-161.ngrok.io";

// Initialize the Ethernet client object
WiFiEspClient client;

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x3F for a 16 chars and 2 line display

//used to wait 60 seconds or 60000 milliseconds before reading sensors in the loop
int period = 5000;
unsigned long time_now = 0;

//used to signal when sensors are done being sent to the server
int doneval = 0;

byte Heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000,
  0b00000
};

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(115200);

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -

  Serial.println("Initializing MAX30102...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //sets lighting on lcd display
  lcd.init();
  lcd.clear();
  lcd.backlight();   // Make sure backlight is on

  // Print a message on LCD.
  lcd.setCursor(1, 0);  //Set cursor to character 1 on line 0
  lcd.print("Connecting to");
  lcd.setCursor(4, 1);  //Move cursor to character 4 on line 1
  lcd.print("Wi-Fi...");

  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network.");
  printWifiStatus();

  lcd.clear();
  // Print a message on LCD.
  lcd.setCursor(2, 0);  //Set cursor to character 1 on line 0
  lcd.print("Connected to");
  lcd.setCursor(5, 1);  //Move cursor to character 4 on line 1
  lcd.print("Wi-Fi!");

  delay(2500);

  lcd.clear();
  // Print a message on LCD.
  lcd.createChar(0, Heart);

  lcd.setCursor(0, 0);  //Set cursor to character 0 on line 0
  lcd.write(byte(0));
  lcd.setCursor(1, 0);  //Set cursor to character 0 on line 0
  lcd.print("BPM:");
  lcd.setCursor(0, 1);  //Set cursor to character 0 on line 0
  lcd.write(byte(0));
  lcd.setCursor(1, 1);  //Set cursor to character 0 on line 0
  lcd.print("Amp:");

  UploadValuesToServer();
}

void loop()
{
  //Constantly checks for incoming bytes from the server, which contains the returned value from the sensor readings.
  while (client.available())
  {
    char c = client.read();
    Serial.write(c);
    //lcd.print(c);
  }

  if ((digitalRead(10) == 1) || (digitalRead(11) == 1)) {
    //Serial.println('!');
  }
  else {
    // send the value of analog input 0:
    Serial.println(analogRead(A0));
    lcd.setCursor(5, 1);  //Set cursor to character 0 on line 0
    lcd.print(analogRead(A0));

    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }


    Serial.print("BPM=");
    Serial.print(beatsPerMinute);
    lcd.setCursor(5, 0);  //Set cursor to character 0 on line 0
    lcd.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
      Serial.print(" No finger?");

    Serial.println();

    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range

    if (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    int i=10;
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);
    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    lcd.setCursor(11, 0);  //Set cursor to character 0 on line 0
    lcd.print("SpO2");
    lcd.setCursor(11, 1);  //Set cursor to character 0 on line 0
    lcd.print(spo2, DEC);
  }

//Once bytes start getting returned from the server, wait 60 seconds before reading data from the sensors again.
if (doneval == 1)
{
  if (millis() > time_now + period + period)
  {
    doneval = 0;

    //resets the client that was used to connect to server and dispose anything that was used
    client.flush();
    client.stop();

    time_now = millis();
    Serial.println("");
    Serial.println("");
    UploadValuesToServer();
  }
}

delay(1000);
}

void UploadValuesToServer()
{
  //Connects to the server once in the setup.
  Serial.println("");
  Serial.println("Starting connection to server...");
  //if you get a connection, report back via serial
  if (client.connect(server, 80))
  {
    Serial.println("Connected to the server.");
    Serial.println("Sending values to server now.");
    // Make a HTTP request with values from sensor.
    client.println("GET /?heartRateAmplitude=" + String(analogRead(A0)) + "&beatsPerMinute=" + String(beatsPerMinute) + "&bloodOxygen=" + String(spo2, DEC) + " HTTP/1.1");
    client.println("Host: b44a-68-194-23-161.ngrok.io");
    client.println();
    Serial.println("");
    Serial.println("");
    Serial.println("");
  }

  doneval = 1;
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
