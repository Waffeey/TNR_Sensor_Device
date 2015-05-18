// Created by Tanner Smith on 4/28 '15.
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "utility/debug.h"
#include "utility/socket.h"
// Temperature and Photocell sensor variables & pin numbers
int tempPin = 1;        //the analog pin the TMP36's Vout (sense) pin is connected
int tempReading;        // the analog reading from the sensor
int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the analog resistor divider
#define aref_voltage 3.3
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
        ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIVIDER);
// WiFi definitions to connect to StickmanPrivate and sensor brokers
#define WLAN_SSID    "StickmanPrivate"
#define WLAN_PASS    "a5730c3a59"
#define WLAN_SECURITY   WLAN_SEC_WPA2
#define ADAFRUIT_USERNAME    "Waffeey"
#define SUBSCRIBE_TEMP    "tanner/temp"
#define SUBSCRIBE_LUX    "tanner/lux"
Adafruit_CC3000_Client client = Adafruit_CC3000_Client();

union superArray {
  byte array[4];
  uint32_t ip;
};

superArray stickman = { 146, 148, 90, 97 };

void callback (char* topic, byte* payload, unsigned int length);

PubSubClient mqttclient("146.148.90.97", 1883, callback, client);

void setup(void)
{
  Serial.begin(115200);
  while(!Serial);
  Serial.println(F("Hello, CC3000!\n")); 
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    for(;;);
  }
 
  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  } 
  
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }
 
  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print("\nAttempting to connect to "); Serial.println(ssid);
  
  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }
 
  /* Display the IP address DNS, Gateway, etc. */  
  while (!displayConnectionDetails()) {
    delay(100);
  }
   
   // connect to the broker, and subscribe to a path
 if (!client.connected()) {
     client = cc3000.connectTCP(superArray.stickman, 1883);
   } else {
     if (mqttclient.connect("ArduinoUnoClient-CC3000-A4")) {
      Serial.println(F("A4 is now online"));
    }
   } 
   
 if(client.connected()) {
    if (mqttclient.connect("ArduinoUnoClient-CC3000-A4")) {
      Serial.println(F("A4 is now online"));
    } 
 }
}
 
void loop(void)
{
// Temperature
  tempReading = analogRead(tempPin);  
  float voltage = tempReading * aref_voltage;
  voltage /= 1024.0;
  int temperatureC = (voltage - 0.33) * 100;
  int temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.println(temperatureF); Serial.println(" degrees F");
  char stringTemp[10];
  dtostrf(photocellReading, 0, 3, stringTemp);
  mqttclient.publish("tanner/temp", stringTemp);
// Lux
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
  dtostrf(photocellReading, 0, 3, stringLux);
  mqttclient.publish("tanner/lux", stringLux);
  mqttclient.loop();
  delay(5000);
} 

uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version; 
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

// Looks for Wifi details and prints them
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  // If statement to determine if cc3000 connects to an IP address
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

