#include <ota.h>


void otasetup()
{

	SPIFFS.begin();

	DEBUG_PRINT("");
	DEBUG_PRINT(F("Example ESPconfig - using ESPAsyncWebServer"));
	DEBUG_PRINT("Version: " ESPMANVERSION);

	Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
	Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());

	settings.begin();



	//  This rewrite is active when the captive portal is working, and redirects the root / to the setup wizard.
	//  This has to go in the main sketch to allow your project to control the root when using ESPManager.
	HTTP.rewrite("/", "/espman/setup.htm").setFilter( [](AsyncWebServerRequest * request) {
		return settings.portal();
	});


	//  then use this rewrite and serve static to serve your index file(s)
	HTTP.rewrite("/", "/index.htm");
	HTTP.serveStatic("/index.htm", SPIFFS, "/index.htm");

	//  rewrite the AJAX loader
	HTTP.rewrite("/images/ajax-loader.gif", "/espman/ajax-loader.gif");

	// Serve favicon from PROGMEM: #include <favicon.h>
	HTTP.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
		AsyncWebServerResponse *response = request->beginResponse_P(200, "image/x-icon", favicon_ico_gz, favicon_ico_gz_len);
		response->addHeader("Content-Encoding", "gzip");
		request->send(response);
	});


	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
	HTTP.begin();


	Serial.print(F("Free Heap: "));
	DEBUG_PRINT(ESP.getFreeHeap());

	Serial.printf("Device Ready IP: %s\n", WiFi.localIP().toString().c_str());

}


void otaloop()
{
	settings.handle();
}
