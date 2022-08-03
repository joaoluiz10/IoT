#include "ArduinoJson.h"
String prepareHtmlPage(String payload);

JsonObject InitializeData();

void manageRequest(String target, WiFiClient *client, String payload);
