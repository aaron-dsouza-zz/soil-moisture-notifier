#include <ESP8266WiFi.h>
#include <arduino.h>
#include "DataToMaker.h"

#define SERIAL_DEBUG // Uncomment this to dissable serial debugging

// define gpio pins here:
#define NUMBER_OF_SENSORS 2 // THIS MUST MATCH THE NUMBER OF SENSORS IN THE SENSOR ARRAY / NO MORE THAN 3

#define FRONT_DOOR_PIN 5 // GPIO5
#define GARAGE_DOOR_PIN 4 // GPIO4
// pin for heatbeat LED
#define HEARTBEAT_PIN 13 //GPIO13
#define LONG_DELAY_MS 900000L // 15mins
//#define LONG_DELAY_MS 30000L // 15mins


// Define program constants
const char* myKey = "bgmvjJlzmxeJi49_0yy5p8";
const char* ssid = "Winterfell"; // your router ssid here
const char* password = "youshallnotpass"; // your router password here

// define program values
int sensors[NUMBER_OF_SENSORS] = {FRONT_DOOR_PIN, GARAGE_DOOR_PIN}; // place your defined sensors in the curly braces
String doorStates[2] = {"Closed", "Open"}; // You can change the LOW / HIGH state strings here
int sensor_pin = A0;

int output_value ;


// declare new maker event with the name "ESP"
DataToMaker event(myKey, "soil_is_dry");

// LEAVE SET

int pvsValues[NUMBER_OF_SENSORS];
bool connectedToWiFI = false;
bool notificationSent = false;

void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  delay(200);
  Serial.println();
#endif

  delay(10); // short delay
  pinMode(HEARTBEAT_PIN, OUTPUT);
  for (int i = 0 ; i < NUMBER_OF_SENSORS ; i++)
  {
    pinMode(sensors[i], INPUT);
    pvsValues[i] = -1; // initialise previous values to -1 to force initial output
  }
  WiFi.mode(WIFI_STA);
  ConnectWifi();
}

void loop() {
  if (wifiConnected)
  {
    if (InsufficientMoisture())
    {
      // Send a notification to IFTTT only if not sent
      if(!notificationSent) 
      {
        debugln("connecting...");
        if (event.connect())
        {
          debugln("Connected To IFTTT");
          event.post();
          debugln("Posted to IFTTT");
          notificationSent = true;
        }
        else
        {
          debugln("Failed To Connect To IFTTT!");
          notificationSent = false;
        }
      }
    }
    else {
      notificationSent = false;
    }
    unsigned long startMillis = millis();
    while (millis() - startMillis < LONG_DELAY_MS)
    {
      delay(1000);
    }
  }
  else
  {
    delay(60 * 1000); // 1 minute delay before trying to re connect
    ConnectWifi();
  }
}

bool ConnectWifi()
{
  // Connect to WiFi network
  debugln();
  debugln();
  debug("Connecting to ");
  debugln(ssid);
  unsigned long startTime = millis();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && startTime + 30 * 1000 >= millis()) {
    delay(500);
    debug(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    debugln("");
    debugln("WiFi connected");
  }
  else
  {
    WiFi.disconnect();
    debugln("");
    debugln("WiFi Timed Out!");
  }
}

bool wifiConnected()
{
  return WiFi.status() == WL_CONNECTED;
}


bool InsufficientMoisture()
{
  bool changed = false;

  output_value= analogRead(sensor_pin);

  debug("Output value:");

  Serial.println(output_value);
 
  output_value = map(output_value,0,300,0,100);

  debug("Mositure : ");

  Serial.print(output_value);

  debugln("%");

  delay(1000);
  changed = output_value < 50;
  if (!changed) 
    debugln("Sufficient moisture Detected");
  else
    debugln("Insufficient moisture Detected");
  return changed;
}


void debug(String message)
{
#ifdef SERIAL_DEBUG
  Serial.print(message);
#endif
}

void debugln(String message)
{
#ifdef SERIAL_DEBUG
  Serial.println(message);
#endif
}

void debugln()
{
#ifdef SERIAL_DEBUG
  Serial.println();
#endif
}
