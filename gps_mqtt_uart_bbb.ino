#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int rxPin = D7;  
const int txPin = D8;

SoftwareSerial mySerial(rxPin, txPin);

SoftwareSerial GPS_SoftSerial(4, 3);

TinyGPSPlus gps;   

volatile float minutes, seconds;
volatile int degree, secs, mins;

// Update these with values suitable for your network.

const char* ssid = "Connect me";
const char* password = "connected123";
const char* mqtt_server = "test.mosquitto.org"; //"5.196.95.208";  // test.mosquitto.org

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(500)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("device/temp", "MQTT Server is Connected");
      // ... and resubscribe
      client.subscribe("device/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);   /* Define baud rate for serial communication */
  GPS_SoftSerial.begin(9600); /* Define baud rate for software serial communication */
  mySerial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  Serial.println("NodeMCU Ready");
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  smartDelay(1000);     /* Generate precise delay of 1ms */
  unsigned long start;
  double lat_val, lng_val, alt_m_val;
  uint8_t hr_val, min_val, sec_val;
  bool loc_valid, alt_valid, time_valid;
  lat_val = gps.location.lat();     /* Get latitude data */
  loc_valid = gps.location.isValid();     /* Check if valid location data is available */
  lng_val = gps.location.lng();     /* Get longtitude data */
  alt_m_val = gps.altitude.meters();      /* Get altitude data in meters */
  alt_valid = gps.altitude.isValid();     /* Check if valid altitude data is available */
  hr_val = gps.time.hour();   /* Get hour */
  min_val = gps.time.minute();      /* Get minutes */
  sec_val = gps.time.second();      /* Get seconds */
  time_valid = gps.time.isValid();  /* Check if valid time data is available */
  if (!loc_valid)
    {         
      Serial.print("Latitude : ");
      Serial.println("*****");
      Serial.print("Longitude : ");
      Serial.println("*****");
    }
    else
      {
        DegMinSec(lat_val);
        Serial.print("Latitude in Decimal Degrees : ");
        Serial.println(lat_val, 6);
        Serial.print("Latitude in Degrees Minutes Seconds : ");
        Serial.print(degree);
        Serial.print("\t");
        Serial.print(mins);
        Serial.print("\t");
        Serial.println(secs);
        DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
        Serial.print("Longitude in Decimal Degrees : ");
        Serial.println(lng_val, 6);
        Serial.print("Longitude in Degrees Minutes Seconds : ");
        Serial.print(degree);
        Serial.print("\t");
        Serial.print(mins);
        Serial.print("\t");
        Serial.println(secs);
      }
      if (!alt_valid)
      {
        Serial.print("Altitude : ");
        Serial.println("*****");
      }
      else
      {
        Serial.print("Altitude : ");
        Serial.println(alt_m_val, 6);   
      }
      if (!time_valid)
      {
        Serial.print("Time : ");
        Serial.println("*****");
      }
      else
      {
        char time_string[32];
        sprintf(time_string, "Time : %02d/%02d/%02d \n", hr_val, min_val, sec_val);
        Serial.print(time_string);   
      }

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
   // value = analogRead(A0)*0.32;

   if (mySerial.available()) {
        snprintf (msg, MSG_BUFFER_SIZE, "Latitude in Decimal Degrees : %lf ; Longitude in Decimal Degrees : %lf", lat_val, lng_val);
    //snprintf (msg2, MSG_BUFFER_SIZE, "Longitude in Decimal Degrees : %lf", lng_val);

   // Serial.print("Publish message: ");
   // Serial.println(msg);
    client.publish("device/temp", msg);
    }
  //   snprintf (msg, MSG_BUFFER_SIZE, "Latitude in Decimal Degrees : %lf ; Longitude in Decimal Degrees : %lf", lat_val, lng_val);
  //   //snprintf (msg2, MSG_BUFFER_SIZE, "Longitude in Decimal Degrees : %lf", lng_val);

  //  // Serial.print("Publish message: ");
  //  // Serial.println(msg);
  //   client.publish("device/temp", msg);
  //  client.publish("device/temp", msg);

    //client.publish("device/temp", lng_val);
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())    /* Encode data read from GPS while data is available on serial port */
     gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)           /* Convert data in decimal degrees into degrees minutes seconds form */
{ 
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}