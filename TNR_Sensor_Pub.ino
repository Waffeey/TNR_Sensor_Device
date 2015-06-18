// Created by Tanner Smith June 2015, for use with a Arduino Leonardo board with an Adafruit cc3000.
#include <PubSubClient.h>
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "utility/socket.h"
#define aref_voltage 3.3
// cc3000 definition goodies for IRQ, VBAT, and CS.
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
        ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
#define WLAN_SSID    "StickmanPrivate"
#define WLAN_PASS    "a5730c3a59"
#define ADAFRUIT_USERNAME    "Waffeey"
#define WLAN_SECURITY   WLAN_SEC_WPA2
// Temperature and Photocell sensor variables & pin numbers.
int tempPin = 1;        // the pin the temperature sensor is connected to
int tempReading;        // the analog reading from the temperature sensor
int photocellPin = 0;     // the pin the photocell sensor is connected to
int photocellReading;     // the analog reading from the photocell
void callback(char* topic, byte* payload, unsigned int length) {
}
uint32_t ip;
Adafruit_CC3000_Client client = Adafruit_CC3000_Client();
PubSubClient mqttclient("146.148.90.97", 1883, callback, client);
void setup(void)
{
  Serial.begin(115200);
  while(!Serial);
  Serial.print(F("Initialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.print(F("Unable to initialise the CC3000! Check your wiring?"));
    for(;;);
  }
  Serial.print(F("Free RAM: ")); Serial.println(getFreeRam(), DEC);
  Serial.println(F("Deleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }
  Serial.println(F("Initialized!"));
  Serial.print(F("Attempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
  while (!cc3000.checkDHCP()) {
    delay(100);
  }
 if (!cc3000.getHostByName("146.148.90.97", &ip)) {
   Serial.println(F("Failed!"));
 }  else {
   Serial.println(F("Connected to DHCP host"));
 }
 if (mqttclient.connect("ArduinoLeoClient-CC3000-A4")) {
   Serial.println(F("Connected to MQTT host"));
   Serial.println(F("Waffeey's Leonardo is now online"));
 } 
}
void loop(void)
{
// Temperature calculation & mqtt publish
  tempReading = analogRead(tempPin);  
  float voltage = tempReading * aref_voltage;
  voltage /= 1024.0;
  int temperatureC = (voltage - 0.33) * 100;
  int temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.print(temperatureF); Serial.println(" degrees F");
  char stringTemp[10];
  dtostrf(temperatureF, 0, 0, stringTemp);
  mqttclient.publish("tanner/temp", stringTemp);
// Lux calculation & mqtt publish
  photocellReading = analogRead(photocellPin);  
  if (photocellReading < 40) {
    Serial.print("Dark at ");
     Serial.println(photocellReading);
  } else if (photocellReading < 90) {
    Serial.print("Dim at ");
     Serial.println(photocellReading);
  } else if (photocellReading < 150) {
    Serial.print("Light at ");
     Serial.println(photocellReading);
  } else if (photocellReading < 200) {
    Serial.print("Bright at ");
     Serial.println(photocellReading);
  } else {
    Serial.print("Very bright at ");
     Serial.println(photocellReading);
  }
  char stringLux[10];
  dtostrf(photocellReading, 0, 0, stringLux);
  mqttclient.publish("tanner/lux", stringLux);
  mqttclient.loop();
  delay(5000);
} 
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpsrv, dnsserv;
  // Can the cc3000 connect to the defined host and if so display the details
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpsrv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.println(F("IP yo: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.println(F("Netmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.println(F("Gateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.println(F("DHCPsrv: ")); cc3000.printIPdotsRev(dhcpsrv);
    Serial.println(F("DNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
