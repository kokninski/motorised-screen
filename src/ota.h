#include <FS.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>

#include <ArduinoOTA.h>
#include <ArduinoJson.h> // required for settings file to make it readable

#include <Hash.h>
#include <ESP8266mDNS.h>
#include <favicon.h>


#include <ESPmanager.h>

AsyncWebServer HTTP(80);

ESPmanager settings(HTTP, SPIFFS);

void otasetup()
void loop()
